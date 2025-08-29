
# pt.py — Feetech SC/STS dashboard (fixed endianness, keys, rescans, race safety)
# Starts on highest COM; clear startup logs; non-blocking idle when no ports.
# Space pause | [/] poll±50ms | ←/→ ±STEP° | ↑/↓ STEP±1° | z/x prev/next COM | R reconnect+rescan | h help | q quit
# Extra: t torque toggle | c one-key center (STS) | r/w raw read/write | k/K reset id/broadcast | a ACTION
import sys, time, struct, argparse, threading, os, platform, re, glob
from collections import defaultdict, deque
import serial
from serial.tools import list_ports

# Prefer a sensible Linux default, fall back on Windows-style when applicable
DEFAULT_PORT = "/dev/ttyACM0"  # e.g., Linux: /dev/ttyACM0 or /dev/ttyUSB0; Windows example: COM9
DEFAULT_BAUD = 1_000_000
DEFAULT_STEP_DEG = 15
POLL_MS_MIN, POLL_MS_MAX = 50, 5000
DISCOVER_CYCLE_MS = 800
DISCOVER_PINGS_WHEN_IDLE = 8      # when no IDs known yet
DISCOVER_PINGS_WHEN_ACTIVE = 2    # when we already track some IDs

HDR = b'\xFF\xFF'
PING, READ, WRITE, REG_WRITE, ACTION, SYNC_WRITE, RESET = 0x01,0x02,0x03,0x04,0x05,0x83,0x06
A_ID=0x05; A_BAUD=0x06
# Feetech SCS/STS registers store 16-bit values little-endian (Low then High)
# See STS3215/SCS15 manual and scservo_sdk helpers (SCS_LOBYTE then SCS_HIBYTE).
A_MIN_ADDR=0x09  # Min_Angle_Limit (2B) then Max_Angle_Limit (2B)
A_MAX_ADDR=0x0B
A_TORQUE=0x28                           # also used by STS “one-key center” (write 128)
A_GOAL=0x2A; A_TIME=0x2C; A_SPEED=0x2E  # each 16-bit, little-endian in memory
A_POS=0x38; A_SPD=0x3A; A_LOAD=0x3C; A_VOLT=0x3E; A_TEMP=0x3F

def checksum(b): return (~(sum(b) & 0xFF)) & 0xFF
def u16_le(lo,hi): return ((hi & 0xFF)<<8) | (lo & 0xFF)
def to_le16(v):    return bytes([v & 0xFF, (v>>8) & 0xFF])

import queue

PORT_SCAN_MAX = 16  # tune if you have higher COM numbers
ENUM_TIMEOUT_S = 1.0

def safe_enum_ports():
    try:
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []

def enumerate_ports():
    ports = []
    t0 = time.time()
    done = queue.Queue(maxsize=1)
    def worker():
        try:
            ps = safe_enum_ports()
        except Exception:
            ps = []
        try:
            done.put_nowait(ps)
        except queue.Full:
            pass
    threading.Thread(target=worker, daemon=True).start()
    while time.time() - t0 < 1.0:
        try:
            return done.get_nowait()
        except queue.Empty:
            time.sleep(0.02)
    # timeout → brute fallback
    return brute_scan_ports(PORT_SCAN_MAX)



def best_port_from_list(ports):
    # Cross-platform best guess: prefer numerically highest within common families.
    # Windows: COMn; Linux: /dev/ttyUSBn, /dev/ttyACMn; macOS: /dev/tty.usbserial*, /dev/tty.usbmodem*
    if not ports:
        return None

    def family_and_num(p: str):
        name = os.path.basename(p)
        # Windows COMn
        if p.upper().startswith("COM"):
            try:
                return ("COM", int(p[3:]))
            except Exception:
                return ("COM", -1)
        # Linux/macOS common patterns
        for fam in ("ttyUSB", "ttyACM", "tty.usbserial", "tty.usbmodem", "cu.usbserial", "cu.usbmodem"):
            if name.startswith(fam):
                m = re.search(r"(\d+)$", name)
                num = int(m.group(1)) if m else -1
                return (fam, num)
        # Fallback: no number
        return (name, -1)

    # Priority order for families (higher is better)
    priority = {
        "COM": 3,
        "ttyUSB": 3,
        "tty.usbserial": 3,
        "cu.usbserial": 3,
        "ttyACM": 2,
        "tty.usbmodem": 2,
        "cu.usbmodem": 2,
    }

    def sort_key(p):
        fam, num = family_and_num(p)
        return (priority.get(fam, 1), num, p)

    return sorted(set(ports), key=sort_key, reverse=True)[0]

def brute_scan_ports(max_com=PORT_SCAN_MAX):
    """Very conservative fallback scan when standard enumeration times out.
    - On Windows: probe COM ports quickly by attempting open/close.
    - On POSIX: use glob to list common device names without opening.
    """
    sysname = platform.system().lower()
    found = []
    if sysname.startswith("win"):
        for n in range(max_com, 0, -1):  # high→low
            name = f"COM{n}"
            try:
                s = serial.Serial(name, baudrate=DEFAULT_BAUD, timeout=0.02, write_timeout=0.02)
                s.close()
                found.append(name)
            except Exception:
                pass
        return found
    # POSIX-style devices (Linux/macOS): just glob known patterns
    patterns = [
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/tty.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.usbmodem*",
    ]
    for pat in patterns:
        found.extend(glob.glob(pat))
    # Deduplicate and sort naturally
    return sorted(set(found), key=lambda p: (os.path.basename(p), p))


def com_num(name: str):
    try:
        if name.upper().startswith("COM"):
            return int(name[3:])
    except:  # noqa: E722
        pass
    # Try trailing digits for POSIX device names
    base = os.path.basename(name)
    m = re.search(r"(\d+)$", base)
    return int(m.group(1)) if m else -1

def list_serial_ports_sorted(desc=True):
    # Keep the function for key handlers; but never block UI at startup.
    try:
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []
    # Order by family and trailing number when present
    def key(x):
        return (com_num(x), x)
    return sorted(set(ports), key=key, reverse=desc)

class SCBus:
    def __init__(self, port, baud, *, read_timeout=0.08, write_timeout=0.2, verbose=False):
        self.port_name = port
        self.baud = baud
        self.ser = None
        self.io_lock = threading.Lock()
        # Use RLock to allow open_on() to call close() safely while holding the lock
        self.port_lock = threading.RLock()
        self.tx_log = deque(maxlen=10)
        self.rx_log = deque(maxlen=10)
        self.read_timeout = read_timeout
        self.write_timeout = write_timeout
        self.verbose = verbose

    @property
    def is_open(self):
        return bool(self.ser and self.ser.is_open)

    def _log(self, pane, msg):
        ts = time.strftime("%H:%M:%S")
        (self.tx_log if pane=="TX" else self.rx_log).appendleft(f"{ts} {msg}")

    def _try_open(self):
        try:
            self.ser = serial.Serial(
                self.port_name,
                baudrate=self.baud,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout,
            )
            self._log("TX", f"OPEN {self.port_name} OK")
        except Exception as e:
            self.ser = None
            self._log("TX", f"OPEN {self.port_name} failed: {e.__class__.__name__}")

    def open_on(self, port_name):
        with self.port_lock:
            self.close()
            self.port_name = port_name
            self._log("TX", f"TRY {port_name}")
            self._try_open()

    def reconnect(self):
        self._log("TX", f"RECONNECT {self.port_name}")
        self.open_on(self.port_name)

    def close(self):
        with self.port_lock:
            try:
                if self.ser and self.ser.is_open:
                    pn = self.port_name
                    self.ser.close()
                    self._log("TX", f"CLOSE {pn}")
            except: pass
            self.ser = None

    def _log_tx(self, data):
        self._log_hex('TX', data)
    def _log_rx(self, data):
        self._log_hex('RX', data)

    def _log_hex(self, pane: str, data: bytes):
        msg = f"{time.strftime('%H:%M:%S')} {data.hex(' ')}"
        if pane == 'TX':
            self.tx_log.appendleft(msg)
        else:
            self.rx_log.appendleft(msg)
        if self.verbose:
            print(f"{pane}: {msg}")

    def _txrx(self, pkt, expect_reply=True, *, hdr_wait=None, timeout_override=None):
        if not self.is_open: return None
        with self.io_lock:
            try:
                t0 = time.time()
                self._log_tx(pkt); self.ser.write(pkt)
                # Optionally override read timeout for this transaction
                orig_timeout = None
                if timeout_override is not None:
                    try:
                        orig_timeout = self.ser.timeout
                        self.ser.timeout = timeout_override
                    except Exception:
                        pass
                if not expect_reply: return None
                # Read status: FF FF ID LEN [ERR|...body...] (LEN includes err/instr..+checksum)
                # Sync to header
                start = time.time()
                wait = 0.12 if hdr_wait is None else float(hdr_wait)
                hdr = b''
                while time.time()-start < wait:
                    ch = self.ser.read(1)
                    if not ch: continue
                    hdr += ch
                    if len(hdr) > 2: hdr = hdr[-2:]
                    if hdr == HDR: break
                if hdr != HDR:
                    if self.verbose:
                        print(f"RX: <no header> after {int((time.time()-t0)*1000)}ms")
                    return None
                header = self.ser.read(2)
                if len(header) < 2:
                    if self.verbose:
                        print("RX: <short header>")
                    return None
                sid, ln = header[0], header[1]
                body = self.ser.read(ln)
                if len(body) != ln:
                    if self.verbose:
                        print(f"RX: <short body> got {len(body)} expected {ln}")
                    return None
                frame = HDR + bytes([sid, ln]) + body
                self._log_rx(frame)
                ck = body[-1]
                calc = checksum([sid, ln] + list(body[:-1]))
                if ck != calc: 
                    self._log("TX", f"CHKERR got {ck:02X} calc {calc:02X}")
                    if self.verbose:
                        print(f"ERR: checksum mismatch rx={ck:02X} calc={calc:02X}")
                    return None
                # status: first byte is error
                return sid, body[0], body[1:-1]
            except Exception as e:
                self._log("TX", f"IOERR {e.__class__.__name__}")
                if self.verbose:
                    print(f"IOERR: {e}")
                return None
            finally:
                if expect_reply and timeout_override is not None and orig_timeout is not None:
                    try:
                        self.ser.timeout = orig_timeout
                    except Exception:
                        pass

    def packet(self, sid, instr, params=b"", expect_reply=True, *, hdr_wait=None, timeout_override=None):
        core = bytes([sid & 0xFF, (2+len(params)) & 0xFF, instr & 0xFF]) + params  # LEN counts instr..checksum
        pkt  = HDR + core + bytes([checksum(core)])
        return self._txrx(pkt, expect_reply=expect_reply, hdr_wait=hdr_wait, timeout_override=timeout_override)

    # simple API
    def ping(self, sid, *, fast=False):
        # Fast mode: short header wait and short read timeout to avoid slowing telemetry when scanning
        if fast:
            return self.packet(sid, PING, expect_reply=True, hdr_wait=0.02, timeout_override=0.02)
        return self.packet(sid, PING)
    def read(self, sid, addr, length):
        r = self.packet(sid, READ, bytes([addr & 0xFF, length & 0xFF]))
        return None if r is None else r[2]   # params only
    def write(self, sid, addr, data, expect_reply=True):
        return self.packet(sid, WRITE, bytes([addr & 0xFF])+bytes(data), expect_reply=expect_reply)
    def reg_write(self, sid, addr, data):
        return self.packet(sid, REG_WRITE, bytes([addr & 0xFF])+bytes(data))
    def action(self):
        core = bytes([0xFE, 2, ACTION]); pkt  = HDR + core + bytes([checksum(core)]); self._txrx(pkt, expect_reply=False)
    def reset(self, sid=0xFE):
        core = bytes([sid & 0xFF, 2, RESET]); pkt = HDR + core + bytes([checksum(core)]); self._txrx(pkt, expect_reply=False)

def read_limits(bus, sid):
    # Read Min_Angle_Limit (2 bytes LE) and Max_Angle_Limit (2 bytes LE)
    b = bus.read(sid, A_MIN_ADDR, 4)
    if not b or len(b)<4: return None
    mn = u16_le(b[0], b[1]); mx = u16_le(b[2], b[3])
    return mn, mx

def read_telemetry(bus, sid):
    # Single 8-byte block read for POS..TEMP to reduce I/O
    b = bus.read(sid, A_POS, 8)
    pos = spd = load = volt = temp = None
    if b and len(b) >= 2:
        pos = u16_le(b[0], b[1])
    if b and len(b) >= 4:
        spd = decode_speed_word(u16_le(b[2], b[3]))
    if b and len(b) >= 6:
        load = u16_le(b[4], b[5])
    if b and len(b) >= 7:
        volt = b[6]
    if b and len(b) >= 8:
        temp = b[7]
    lim = read_limits(bus, sid) or (None, None)
    return {"pos":pos, "spd":spd, "load":load, "volt":volt, "temp":temp, "min":lim[0], "max":lim[1]}

def ticks_per_degree(minmax):
    if not minmax or None in minmax: 
        # fall back to STS (4096/360) which is safer for step sizing
        return 4095/360.0
    rng = max(1, (minmax[1]-minmax[0]))
    return (1023/200.0) if rng <= 1100 else (4095/360.0)  # SC≈200°, STS≈360°

def step_deg(bus, sid, deg, limits):
    t = bus.read(sid, A_POS, 2)
    if not t or len(t)<2: return False
    cur = u16_le(t[0], t[1])
    dt  = int(round(deg * ticks_per_degree(limits)))
    goal = cur + dt
    if limits and None not in limits:
        goal = min(max(goal, limits[0]), limits[1])
    # pos, time, speed (all 16-bit little-endian)
    data = to_le16(goal) + to_le16(0) + to_le16(0)
    return bus.write(sid, A_GOAL, data, expect_reply=True) is not None

def torque(bus, sid, on=True):
    # Write without requiring an ACK to avoid false negatives on some devices
    bus.write(sid, A_TORQUE, [1 if on else 0], expect_reply=False)
def sts_one_key_center(bus, sid): bus.write(sid, A_TORQUE, [128])  # STS only (sets current to 2048)

def decode_speed_word(v: int) -> int:
    """Decode the signed-ish speed word from telemetry.
    0..16384 -> 0..16384; 16385..32768 -> -16384..0 (wrap-around).
    """
    return v if v <= 16384 else (v - 32768)

HELP_MIN = "Space pause | [/] poll±50ms | Sel:1..9 | ←/→ ±STEP° | ↑/↓ next/prev ID | +/- STEP±1° | r/w | t | c | a | z/x prev/next COM | R reconnect+rescan | k/K reset | h | q"
HELP_FULL = [
    "Space pause; [/] poll 50..2000ms; ↑/↓ select next/prev ID; +/- step 1..90°; ←/→ move ±STEP°",
    "z/x prev/next COM (graceful close/open); R reconnect + immediate full ID rescan",
    "r read 'addr,len'; w write 'addr bytes...'; t torque; c center(128@0x28, STS)",
    "k reset@ID; K reset broadcast (dangerous); a broadcast ACTION",
]

def run_ui(start_port, baud):
    try:
        import curses
    except ImportError:
        # Helpful hint by platform
        if platform.system().lower().startswith("win"):
            print("Missing curses. On Windows: pip install windows-curses")
        else:
            print("Missing curses. Install your OS ncurses dev packages.")
        sys.exit(1)

    # Non-blocking startup: show UI immediately; enumerate ports asynchronously
    bus = SCBus(start_port, baud)
    bus._log("TX", "Startup: enumerating ports...")

    ports = enumerate_ports()
    # cache used by z/x handlers; refreshed asynchronously
    ports_cache = list(ports)

    def enumerator():
        nonlocal ports_cache
        try:
            ports_cache = list_serial_ports_sorted(desc=True)
        except Exception:
            ports_cache = ports_cache or []
    if ports:
        best = best_port_from_list(ports)
        bus._log("TX", f"Detected ports: {ports}")
        bus.open_on(best)
    else:
        bus._log("TX", "No ports detected; idling on requested port")
        bus.open_on(start_port)
        
    # Shared state (guard with a lock)
    state_lock = threading.Lock()
    selected = 1
    step_deg_value = DEFAULT_STEP_DEG
    poll_ms = 200
    paused = [False]
    known_ids = set()
    telemetry = defaultdict(dict)
    limits_cache = {}
    show_help = [False]
    discover_next = 1
    last_discover_t = 0
    stop = {"f": False}

    def seed_scan():
        if bus.is_open:
            with state_lock:
                known_ids.clear()
            for sid in range(1,16):
                if bus.ping(sid) is not None:
                    with state_lock: known_ids.add(sid)

    seed_scan()

    def full_rescan():
        # fast, blocking scan of all IDs (kept short; logs once done)
        cnt = 0
        with state_lock: known_ids.clear()
        for sid in range(1,254):
            if not bus.is_open: break
            if bus.ping(sid) is not None:
                with state_lock: known_ids.add(sid); cnt += 1
        bus._log("TX", f"RESCAN found {cnt} servo(s)")

    def poller():
        nonlocal discover_next, last_discover_t
        while not stop["f"]:
            if not paused[0] and bus.is_open:
                # read telemetry for known
                with state_lock:
                    ids = sorted(list(known_ids))
                for sid in ids:
                    t = read_telemetry(bus, sid)
                    if t:
                        with state_lock: telemetry[sid] = t
                        if sid not in limits_cache and t.get("min") is not None:
                            with state_lock: limits_cache[sid] = (t["min"], t["max"])
                # incremental discovery
                now = time.time()*1000.0
                if now - last_discover_t >= DISCOVER_CYCLE_MS:
                    # Keep discovery lightweight to not starve telemetry I/O
                    pings_per_cycle = (
                        DISCOVER_PINGS_WHEN_IDLE if not known_ids else DISCOVER_PINGS_WHEN_ACTIVE
                    )
                    for _ in range(pings_per_cycle):
                        sid = discover_next
                        discover_next = 1 if discover_next>=253 else (discover_next+1)
                        r = bus.ping(sid, fast=True)
                        if r is not None:
                            with state_lock: known_ids.add(sid)
                    last_discover_t = now
                time.sleep(max(0.01, poll_ms/1000.0))
            else:
                time.sleep(0.05)

    th = threading.Thread(target=poller, daemon=True); th.start()

    def draw(stdscr):
        nonlocal selected, step_deg_value, poll_ms
        stdscr.nodelay(True)
        try:
            curses = __import__("curses"); curses.curs_set(0)
        except: pass

        def get_sorted_ids():
            with state_lock:
                return sorted(list(known_ids))

        def cycle_port(offset: int):
            """Cycle through cached ports by offset (+1 next, -1 prev)."""
            if not ports_cache:
                threading.Thread(target=enumerator, daemon=True).start()
            plist = ports_cache or [bus.port_name]
            cur = bus.port_name
            idx = plist.index(cur) if cur in plist else 0
            newp = plist[(idx + offset) % len(plist)]
            bus.open_on(newp)
            seed_scan()

        # Key handlers (return True to continue, False to exit loop)
        def quit_app():
            return False
        def toggle_help():
            show_help[0] = not show_help[0]; return True
        def toggle_pause():
            paused[0] = not paused[0]; return True
        def dec_poll():
            nonlocal poll_ms; poll_ms = max(POLL_MS_MIN, poll_ms - 50); return True
        def inc_poll():
            nonlocal poll_ms; poll_ms = min(POLL_MS_MAX, poll_ms + 50); return True
        def inc_step():
            nonlocal step_deg_value; step_deg_value = min(90, step_deg_value + 1); return True
        def dec_step():
            nonlocal step_deg_value; step_deg_value = max(1,  step_deg_value - 1); return True
        def prev_id():
            nonlocal selected
            ids = get_sorted_ids()
            if ids:
                if selected not in ids:
                    selected = ids[0]
                else:
                    idx = ids.index(selected); selected = ids[(idx - 1) % len(ids)]
            else:
                selected = 31 if selected <= 1 else selected - 1
            return True
        def next_id():
            nonlocal selected
            ids = get_sorted_ids()
            if ids:
                if selected not in ids:
                    selected = ids[0]
                else:
                    idx = ids.index(selected); selected = ids[(idx + 1) % len(ids)]
            else:
                selected = 1 if selected >= 31 else selected + 1
            return True
        def move_left():
            if bus.is_open:
                step_deg(bus, selected, -step_deg_value, lims.get(selected, read_limits(bus, selected)))
            return True
        def move_right():
            if bus.is_open:
                step_deg(bus, selected, +step_deg_value, lims.get(selected, read_limits(bus, selected)))
            return True
        def torque_toggle():
            if bus.is_open:
                cur = bus.read(selected, A_TORQUE, 1)
                enabled = bool(cur and len(cur) >= 1 and cur[0] != 0)
                torque(bus, selected, on=not enabled)
            return True
        def center():
            if bus.is_open:
                sts_one_key_center(bus, selected)
            return True
        def read_raw():
            if not bus.is_open: return True
            try:
                import curses as _c
            except Exception:
                return True
            _c.echo(); stdscr.addstr(1,0, "Read addr,len ? e.g. 0x38,2 → "); stdscr.clrtoeol()
            s = stdscr.getstr(1,34,40).decode().strip(); _c.noecho()
            try:
                a,l = [x.strip() for x in s.split(",")]; addr=int(a,0); ln=int(l,0)
                _ = bus.read(selected, addr, ln)
                bus.tx_log.appendleft(f"{time.strftime('%H:%M:%S')} READ S{selected} @{addr} len {ln}")
            except Exception as e:
                bus.tx_log.appendleft(f"{time.strftime('%H:%M:%S')} READ ERR {e}")
            return True
        def write_raw():
            if not bus.is_open: return True
            try:
                import curses as _c
            except Exception:
                return True
            _c.echo(); stdscr.addstr(1,0, "Write addr bytes... ? e.g. 0x2A 0x08 0x00 0x00 0x00 0x03 0xE8 → "); stdscr.clrtoeol()
            s = stdscr.getstr(1,70,80).decode().strip(); _c.noecho()
            try:
                parts = s.split(); addr=int(parts[0],0); data=[int(x,0) for x in parts[1:]]
                r = bus.write(selected, addr, data, expect_reply=True)
                bus.tx_log.appendleft(f"{time.strftime('%H:%M:%S')} WRITE S{selected} @{addr} {len(data)}B -> {'OK' if r else 'NO-ACK'}")
            except Exception as e:
                bus.tx_log.appendleft(f"{time.strftime('%H:%M:%S')} WRITE ERR {e}")
            return True
        def do_action():
            if bus.is_open: bus.action(); return True
        def reset_sel():
            if bus.is_open: bus.reset(selected); return True
        def reset_all():
            if bus.is_open: bus.reset(0xFE); return True
        def reconnect_and_rescan():
            bus.reconnect(); threading.Thread(target=full_rescan, daemon=True).start(); return True
        def prev_port():
            cycle_port(+1); return True
        def next_port():
            cycle_port(-1); return True

        # Map keys to handlers (include numeric fallbacks for curses constants)
        KEY_UP = 259
        KEY_DOWN = 258
        KEY_LEFT = 260
        KEY_RIGHT = 261
        try:
            import curses as _ck
            KEY_UP = getattr(_ck, 'KEY_UP', KEY_UP)
            KEY_DOWN = getattr(_ck, 'KEY_DOWN', KEY_DOWN)
            KEY_LEFT = getattr(_ck, 'KEY_LEFT', KEY_LEFT)
            KEY_RIGHT = getattr(_ck, 'KEY_RIGHT', KEY_RIGHT)
        except Exception:
            pass
        keymap = {
            ord('q'): quit_app, ord('Q'): quit_app,
            ord('h'): toggle_help, ord('H'): toggle_help,
            ord(' '): toggle_pause,
            ord('['): dec_poll, ord(']'): inc_poll,
            ord('+'): inc_step, ord('='): inc_step,
            ord('-'): dec_step, ord('_'): dec_step,
            KEY_UP: prev_id, 259: prev_id,
            KEY_DOWN: next_id, 258: next_id,
            KEY_LEFT: move_left, 260: move_left,
            KEY_RIGHT: move_right, 261: move_right,
            ord('t'): torque_toggle, ord('T'): torque_toggle,
            ord('c'): center, ord('C'): center,
            ord('r'): read_raw,
            ord('w'): write_raw, ord('W'): write_raw,
            ord('a'): do_action, ord('A'): do_action,
            ord('k'): reset_sel,
            ord('K'): reset_all,
            ord('R'): reconnect_and_rescan,
            ord('z'): prev_port, ord('Z'): prev_port,
            ord('x'): next_port, ord('X'): next_port,
        }

        while True:
            stdscr.erase()
            h,w = stdscr.getmaxyx()
            port_label = bus.port_name + (" (OPEN)" if bus.is_open else " (closed)")
            ids_snapshot = get_sorted_ids()
            header = f"Port:{port_label} @ {baud} | IDs:{ids_snapshot or '-'} | Sel:{selected} | STEP:{step_deg_value}° | Poll:{poll_ms}ms | {'PAUSED' if paused[0] else 'RUN'}   {HELP_MIN}"
            stdscr.addstr(0, 0, header[:w-1])

            stdscr.addstr(2, 0, f"{'ID':>2} | {'POS dec/hex':>16} | {'SPD':>8} | {'LOAD':>8} | {'VOLT':>6} | {'TEMP':>6} | {'MIN..MAX':>15}")
            stdscr.addstr(3, 0, "-"*(w-1))
            row = 4
            ids_to_show = ids_snapshot if ids_snapshot else list(range(1,6))
            with state_lock:
                tel = dict(telemetry)
                lims = dict(limits_cache)
            for sid in ids_to_show:
                t = tel.get(sid, {})
                pos  = t.get("pos"); spd=t.get("spd"); load=t.get("load"); volt=t.get("volt"); temp=t.get("temp")
                mn, mx = t.get("min"), t.get("max")
                pos_s = f"{pos if pos is not None else '':>5} / {('0x%04X'%pos) if pos is not None else ''}"
                spd_s = ("" if spd is None else (f"{spd}"))
                mark = ">" if sid==selected else " "
                stdscr.addstr(row, 0, f"{mark}{sid:>2} | {pos_s:>16} | {spd_s:>8} | {str(load or ''):>8} | {str(volt or ''):>6} | {str(temp or ''):>6} | {'' if mn is None else mn}..{'' if mx is None else mx}")
                row+=1
                if row >= h-14: break

            row += 1
            stdscr.addstr(row, 0, "TX (last 10):")
            midcol = max(35, w//2)
            stdscr.addstr(row, midcol, "RX (last 10):")
            row += 1
            max_lines = min(10, h - row - (len(HELP_FULL)+2 if show_help[0] else 2))
            for i in range(max_lines):
                tx = bus.tx_log[i] if i < len(bus.tx_log) else ""
                rx = bus.rx_log[i] if i < len(bus.rx_log) else ""
                stdscr.addstr(row+i, 0, tx[:midcol-1].ljust(midcol-1))
                stdscr.addstr(row+i, midcol, rx[:w-midcol-1].ljust(w-midcol-1))

            if show_help[0]:
                box_top = row + max_lines + 1
                if box_top < h-2:
                    stdscr.addstr(box_top, 0, "-"*(w-1))
                    for i, line in enumerate(HELP_FULL, start=1):
                        if box_top+i >= h-1: break
                        stdscr.addstr(box_top+i, 0, line[:w-1])

            stdscr.refresh()

            try: ch = stdscr.getch()
            except: ch = -1
            if ch == -1:
                time.sleep(0.02); continue

            # Digits 1..9 select ID directly
            if ch in tuple(ord(str(d)) for d in range(1,10)):
                selected = int(chr(ch))
                continue

            h = keymap.get(ch)
            if h is None:
                continue
            if h() is False:
                break

        stop["f"]=True
        bus.close()

    import curses
    curses.wrapper(draw)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--list-ports", action="store_true", help="List detected serial ports (ttyACM only on Linux) and exit")
    ap.add_argument("--probe", action="store_true", help="Open port and ping IDs non-interactively with raw I/O logs, then exit")
    ap.add_argument("--probe-timeout", type=float, default=8.0, help="Max seconds to spend probing before aborting")
    ap.add_argument("--ids", default="1-31", help="ID set to probe, e.g. '1-8' or '1,2,5'")
    args = ap.parse_args()

    if args.list_ports:
        run_list_ports()
        return

    if args.probe:
        run_probe(args)
        return

    run_ui(args.port, args.baud)

def run_list_ports():
    ports = list_serial_ports_sorted(desc=True)
    if platform.system().lower() == 'linux':
        ports = [p for p in ports if p.startswith('/dev/ttyACM')]
    if not ports:
        print("No serial ports detected.")
    else:
        print("Detected ports (best first):")
        for p in ports:
            print(f" - {p}")

def parse_ids(spec: str):
    s = (spec or '').strip()
    if '-' in s:
        a, b = s.split('-', 1)
        return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(',') if x.strip()]

def run_probe(args):
    ids = parse_ids(args.ids)
    ports = list_serial_ports_sorted(desc=True)
    if platform.system().lower() == 'linux':
        ports = [p for p in ports if p.startswith('/dev/ttyACM')]
    port = best_port_from_list(ports) if ports else args.port
    print(f"Probing on port: {port} @ {args.baud}")
    bus = SCBus(port, args.baud, read_timeout=0.06, write_timeout=0.15, verbose=True)
    bus.open_on(port)
    if not bus.is_open:
        print("Failed to open port; aborting.")
        return
    start = time.time()
    found = []
    for sid in ids:
        if time.time() - start > args.probe_timeout:
            print(f"Probe timeout reached ({args.probe_timeout}s). Aborting scan.")
            break
        core = bytes([sid & 0xFF, 2, PING])
        pkt  = HDR + core + bytes([checksum(core)])
        print(f"PING S{sid}: {pkt.hex(' ')}")
        r = bus.packet(sid, PING)
        if r is None:
            print(f" -> S{sid}: no reply")
        else:
            sidr, err, params = r
            print(f" -> S{sidr}: err=0x{err:02X} params={params.hex(' ')}")
            found.append(sidr)
    bus.close()
    print(f"Found IDs: {found if found else 'None'}")

if __name__=="__main__":
    main()
