#!/usr/bin/env python3

import argparse, os, platform, re, sys, threading, time
from collections import defaultdict, deque
from queue import Queue, Empty, Full
from dataclasses import dataclass

try:
    # Prefer normal import if namespace packages resolve
    from phosphobot.scripts.feetech.feetech import FeetechMotorsBus
except Exception:
    # Fallback: append repo root to path if running from within workspace
    ROOT = os.path.dirname(os.path.abspath(__file__))
    CAND = os.path.join(ROOT, "phosphobot", "scripts", "feetech")
    if os.path.isdir(CAND):
        # ensure both the feetech folder (for motor_utils) and its parents are importable
        sys.path.insert(0, CAND)  # so 'from motor_utils import ...' works
        sys.path.insert(0, os.path.dirname(CAND))  # add "phosphobot/scripts"
        sys.path.insert(0, os.path.join(ROOT, "phosphobot"))
        sys.path.insert(0, ROOT)
    # retry
    from phosphobot.scripts.feetech.feetech import FeetechMotorsBus

from serial.tools import list_ports

# Defaults
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_STEP_DEG = 15
POLL_MS_MIN, POLL_MS_MAX = 50, 5000
DISCOVER_CYCLE_MS = 800

def list_serial_ports_sorted(desc=True):
    try:
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []

    def com_num(name):
        try:
            if name.upper().startswith("COM"):
                return int(name[3:])
        except Exception:
            pass
        base = os.path.basename(name)
        m = re.search(r"(\d+)$", base)
        return int(m.group(1)) if m else -1

    return sorted(set(ports), key=lambda x: (com_num(x), x), reverse=desc)

def best_port_from_list(ports):
    ports = list(ports)
    if not ports:
        return None

    def family_and_num(p):
        name = os.path.basename(p)
        if p.upper().startswith("COM"):
            try:
                return ("COM", int(p[3:]))
            except Exception:
                return ("COM", -1)
        for fam in ("ttyUSB", "ttyACM", "tty.usbserial", "tty.usbmodem", "cu.usbserial", "cu.usbmodem"):
            if name.startswith(fam):
                m = re.search(r"(\d+)$", name)
                num = int(m.group(1)) if m else -1
                return (fam, num)
        return (name, -1)

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


# Simple, focused Feetech comms wrapper
@dataclass
class FeetechBus:
    port: str
    baud: int
    model: str = "sts3215"  # default model name used for scanning and I/O

    def __post_init__(self):
        self._bus = None
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return bool(self._bus and getattr(self._bus, "is_connected", False))

    def open(self):
        with self._lock:
            if self.is_open:
                return
            self._bus = FeetechMotorsBus(port=self.port, motors={"_dummy": (1, self.model)})
            self._bus.connect()
            for fn in (
                lambda: self._bus.set_bus_baudrate(self.baud),
                # Tighten SDK packet timeout to keep UI responsive
                lambda: getattr(getattr(self._bus, "port_handler", None), "setPacketTimeoutMillis", lambda _:_)(50),
            ):
                try: fn()
                except Exception: pass

    def close(self):
        with self._lock:
            try:
                if self._bus and self._bus.is_connected:
                    self._bus.disconnect()
            finally:
                self._bus = None

    def reconnect(self):
        self.close()
        self.open()

    # Basic ops using read/write by ids (no prior motor config required)
    def ping(self, sid: int) -> bool:
        if not self.is_open:
            return False
        try:
            val = self._bus.read_with_motor_ids([self.model], [sid], "ID", num_retry=1)
            return bool(val and int(val[0]) == sid)
        except Exception:
            return False

    def read_limits(self, sid):
        try:
            mn = self._bus.read_with_motor_ids([self.model], [sid], "Min_Angle_Limit", num_retry=1)[0]
            mx = self._bus.read_with_motor_ids([self.model], [sid], "Max_Angle_Limit", num_retry=1)[0]
            return int(mn), int(mx)
        except Exception:
            return None

    def read_telemetry(self, sid):
        """Read POS..TEMP using a single 8B block if possible; fall back to individual reads."""
        res = {}
        blk = self.read_block(sid, 0x38, 8, num_retry=1)
        if blk and len(blk) == 8:
            try:
                pos = int.from_bytes(blk[0:2], "little")
                spd_raw = int.from_bytes(blk[2:4], "little")
                spd = spd_raw if spd_raw <= 16384 else (spd_raw - 32768)
                load = int.from_bytes(blk[4:6], "little")
                volt = blk[6]
                temp = blk[7]
                res.update(pos=pos, spd=spd, load=load, volt=volt, temp=temp)
                return res
            except Exception:
                pass
        # Fallback path: individual reads
        try:
            pos = self._bus.read_with_motor_ids([self.model], [sid], "Present_Position", num_retry=1)[0]
            res["pos"] = int(pos)
        except Exception:
            pass
        try:
            spd = self._bus.read_with_motor_ids([self.model], [sid], "Present_Speed", num_retry=1)[0]
            res["spd"] = int(spd)
        except Exception:
            pass
        try:
            load = self._bus.read_with_motor_ids([self.model], [sid], "Present_Load", num_retry=1)[0]
            res["load"] = int(load)
        except Exception:
            pass
        try:
            volt = self._bus.read_with_motor_ids([self.model], [sid], "Present_Voltage", num_retry=1)[0]
            res["volt"] = int(volt)
        except Exception:
            pass
        try:
            temp = self._bus.read_with_motor_ids([self.model], [sid], "Present_Temperature", num_retry=1)[0]
            res["temp"] = int(temp)
        except Exception:
            pass
        return res

    def torque(self, sid: int, on: bool):
        if not self.is_open:
            return
        try:
            self._bus.write_with_motor_ids([self.model], [sid], "Torque_Enable", [1 if on else 0], num_retry=1)
        except Exception:
            pass

    def center_sts(self, sid: int):
        # STS one-key center: write 128 to Torque_Enable (mirrors robo.py behavior)
        if not self.is_open:
            return
        try:
            self._bus.write_with_motor_ids([self.model], [sid], "Torque_Enable", [128], num_retry=1)
        except Exception:
            pass

    def step_deg(self, sid, deg, limits):
        if not self.is_open:
            return False
        try:
            # Read current position once; avoid extra torque writes for throughput
            cur = int(self._bus.read_with_motor_ids([self.model], [sid], "Present_Position", num_retry=1)[0])
        except Exception:
            return False
        # Assume 4096 ticks over full 360° for STS/SCS here for simplicity
        dt = int(round(deg * (4096.0 / 360.0)))
        goal = cur + dt
        if limits and None not in limits:
            lo, hi = limits
            goal = min(max(goal, lo), hi)
        try:
            self._bus.write_with_motor_ids([self.model], [sid], "Goal_Position", [goal], num_retry=1)
            return True
        except Exception:
            return False

    def read_block(self, sid: int, addr: int, length: int, *, num_retry: int = 1):
        """Efficient contiguous block read using GroupSyncRead for a single ID.
        Returns bytes or None on failure. Falls back to None if SDK unavailable.
        """
        if not self.is_open:
            return None
        try:
            import scservo_sdk as scs  # type: ignore
            group = scs.GroupSyncRead(self._bus.port_handler, self._bus.packet_handler, addr, length)  # type: ignore[attr-defined]
            group.addParam(sid)
            comm = None
            for _ in range(max(1, num_retry)):
                comm = group.txRxPacket()
                if comm == scs.COMM_SUCCESS:
                    break
            if comm != scs.COMM_SUCCESS:
                return None
            val = group.getData(sid, addr, length)
            return int(val).to_bytes(length, byteorder="little", signed=False)
        except Exception:
            return None

    def read_block_multi(self, sids: list[int], addr: int, length: int, *, num_retry: int = 1):
        """Batch read a contiguous block for multiple IDs in one GroupSyncRead.
        Returns dict[sid] = bytes(length) for successes; missing sids are omitted.
        """
        out = {}
        if not self.is_open or not sids:
            return out
        try:
            import scservo_sdk as scs  # type: ignore
            group = scs.GroupSyncRead(self._bus.port_handler, self._bus.packet_handler, addr, length)  # type: ignore[attr-defined]
            for sid in sids:
                group.addParam(sid)
            comm = None
            for _ in range(max(1, num_retry)):
                comm = group.txRxPacket()
                if comm == scs.COMM_SUCCESS:
                    break
            if comm != scs.COMM_SUCCESS:
                return out
            for sid in sids:
                val = group.getData(sid, addr, length)
                try:
                    out[sid] = int(val).to_bytes(length, byteorder="little", signed=False)
                except Exception:
                    pass
            return out
        except Exception:
            return out

    def read_field_multi(self, sids: list[int], data_name: str, *, num_retry: int = 1):
        """Read a single named field for multiple IDs; robust per-ID to avoid group failures.
        Returns dict[sid] = int for successes; missing sids omitted.
        """
        out = {}
        if not self.is_open or not sids:
            return out
        for sid in sids:
            try:
                v = self._bus.read_with_motor_ids([self.model], [sid], data_name, num_retry=num_retry)[0]
                out[sid] = int(v)
            except Exception:
                continue
        return out


HELP_MIN = (
    "Space pause | [/] poll±50ms | Sel:1..9 | ↑/↓ select ID | ←/→ ±STEP° | z/x prev/next COM | R reconnect | h help | q quit"
)
HELP_FULL = [
    "Space pause; [/] poll 50..2000ms; ↑/↓ select prev/next ID (wrap); ←/→ move ±STEP°",
    "z/x prev/next COM; R reconnect",
    "t torque toggle; c center (STS only)",
]


def parse_ids_arg(s):
    s = (s or '').strip()
    if not s: return []
    if '-' in s:
        a,b = s.split('-',1); return list(range(int(a), int(b)+1))
    return [int(x) for x in s.split(',') if x.strip()]


def run_ui(start_port, baud, model, ui_ids, discover_ids):
    try:
        import curses
    except ImportError:
        if platform.system().lower().startswith("win"):
            print("Missing curses. On Windows: pip install windows-curses")
        else:
            print("Missing curses. Install your OS ncurses dev packages.")
        sys.exit(1)

    bus = FeetechBus(start_port, baud, model=model)
    tx_log = deque(maxlen=12)

    # Ports
    ports = list_serial_ports_sorted(desc=True)
    best = best_port_from_list(ports) or start_port
    bus.port = best
    try:
        bus.open()
        tx_log.appendleft(f"OPEN {best} OK @ {baud}")
    except Exception as e:
        tx_log.appendleft(f"OPEN {best} failed: {e.__class__.__name__}")

    # Optional one-shot discovery to populate IDs before starting UI
    found_ids = []
    if discover_ids:
        for sid in discover_ids:
            if bus.ping(sid):
                found_ids.append(sid)
        tx_log.appendleft(f"DISCOVER found {len(found_ids)} servo(s)")

    # Shared state
    state_lock = threading.Lock()
    base_ids = found_ids if found_ids else (ui_ids if ui_ids else list(range(1, 7)))
    selected = base_ids[0] if base_ids else 1
    step_deg_value = DEFAULT_STEP_DEG
    poll_ms = 200
    paused = [False]
    # Start from requested IDs, then quickly verify which ones respond to avoid group timeouts
    verified_ids = []
    for sid in base_ids:
        try:
            if bus.ping(sid):
                verified_ids.append(sid)
        except Exception:
            pass
    if verified_ids:
        tx_log.appendleft(f"INIT verified IDs: {verified_ids}")
    else:
        tx_log.appendleft("INIT no IDs verified; using requested IDs")
    known_ids = set(verified_ids if verified_ids else base_ids)
    telemetry = defaultdict(dict)
    limits_cache = {}
    show_help = [False]
    stop = {"f": False}
    actions: Queue = Queue(maxsize=128)
    torque_state = defaultdict(lambda: None)
    def put(act):
        try:
            actions.put_nowait(act)
        except Full:
            try:
                tx_log.appendleft("QUEUE FULL: "+str(act[0]))
            except Exception:
                pass

    def poller():
        while not stop["f"]:
            if not paused[0] and bus.is_open:
                # Drain a few actions per loop
                for _ in range(32):  # drain more actions to keep controls snappy
                    try:
                        act = actions.get_nowait()
                    except Empty:
                        break
                    kind = act[0]
                    try:
                        if kind == "move":
                            _, sid, delta_deg = act
                            bus.step_deg(sid, delta_deg, limits_cache.get(sid))
                        elif kind == "toggle_torque":
                            _, sid = act
                            try:
                                cur = bus._bus.read_with_motor_ids([model], [sid], "Torque_Enable", num_retry=1)[0]  # type: ignore[attr-defined]
                                new_state = 0 if int(cur) != 0 else 1
                            except Exception:
                                prev = torque_state[sid]
                                new_state = 0 if (prev and prev != 0) else 1
                            torque_state[sid] = new_state
                            bus.torque(sid, on=bool(new_state))
                            tx_log.appendleft(f"TORQUE S{sid} -> {'ON' if new_state else 'OFF'}")
                        elif kind == "center":
                            _, sid = act
                            bus.center_sts(sid)
                            tx_log.appendleft(f"CENTER S{sid} (STS)")
                        elif kind == "reconnect":
                            bus.reconnect()
                            tx_log.appendleft("RECONNECT OK")
                        elif kind == "switch_port":
                            _, newp = act
                            bus.port = newp
                            bus.reconnect()
                            tx_log.appendleft(f"SWITCH {newp}")
                    except Exception as e:
                        tx_log.appendleft(f"ACT ERR {kind}: {e}")

                with state_lock:
                    ids = sorted(list(known_ids))
                # Cache limits once per ID (out of band)
                for sid in ids:
                    if sid not in limits_cache:
                        lim = bus.read_limits(sid)
                        if lim and None not in lim:
                            with state_lock:
                                limits_cache[sid] = lim

                # Stagger telemetry reads to minimize bus congestion
                if not hasattr(poller, "_cycle"):
                    poller._cycle = 0  # type: ignore[attr-defined]
                cycle = poller._cycle  # type: ignore[attr-defined]
                if ids:
                    if cycle == 0:
                        mp = bus.read_field_multi(ids, "Present_Position", num_retry=1)
                        if mp:
                            with state_lock:
                                for sid, v in mp.items():
                                    telemetry[sid].update(pos=v)
                    elif cycle == 1:
                        mp = bus.read_field_multi(ids, "Present_Speed", num_retry=1)
                        if mp:
                            with state_lock:
                                for sid, v in mp.items():
                                    telemetry[sid].update(spd=(v if v <= 16384 else (v - 32768)))
                    elif cycle == 2:
                        mp = bus.read_field_multi(ids, "Present_Load", num_retry=1)
                        if mp:
                            with state_lock:
                                for sid, v in mp.items():
                                    telemetry[sid].update(load=v)
                    else:
                        mp_v = bus.read_field_multi(ids, "Present_Voltage", num_retry=1)
                        mp_t = bus.read_field_multi(ids, "Present_Temperature", num_retry=1)
                        with state_lock:
                            for sid, v in mp_v.items():
                                telemetry[sid].update(volt=v)
                            for sid, v in mp_t.items():
                                telemetry[sid].update(temp=v)
                    poller._cycle = (cycle + 1) % 4  # type: ignore[attr-defined]
                time.sleep(max(0.01, poll_ms / 1000.0))
            else:
                time.sleep(0.05)

    threading.Thread(target=poller, daemon=True).start()

    def draw(stdscr):
        nonlocal selected, step_deg_value, poll_ms
        stdscr.nodelay(True)
        try:
            curses = __import__("curses"); curses.curs_set(0)
        except Exception:
            pass

        while True:
            stdscr.erase()
            h, w = stdscr.getmaxyx()
            port_label = f"{bus.port} ({'OPEN' if bus.is_open else 'closed'})"
            with state_lock:
                ids_snapshot = sorted(list(known_ids))
                tel = dict(telemetry)
                lims = dict(limits_cache)
            header = (
                f"Port:{port_label} @ {baud} | Model:{model} | IDs:{ids_snapshot or '-'} | "
                f"Sel:{selected} | STEP:{step_deg_value}° | Poll:{poll_ms}ms | Q:{actions.qsize()} | "
                f"{'PAUSED' if paused[0] else 'RUN'}   {HELP_MIN}"
            )
            stdscr.addstr(0, 0, header[: w - 1])

            stdscr.addstr(2, 0, f"{'ID':>2} | {'POS dec/hex':>16} | {'SPD':>8} | {'LOAD':>8} | {'VOLT':>6} | {'TEMP':>6} | {'MIN..MAX':>15}")
            stdscr.addstr(3, 0, "-" * (w - 1))
            row = 4
            ids_to_show = ids_snapshot if ids_snapshot else list(range(1, 6))
            for sid in ids_to_show:
                t = tel.get(sid, {})
                pos = t.get("pos")
                spd = t.get("spd")
                load = t.get("load")
                volt = t.get("volt")
                temp = t.get("temp")
                mn, mx = t.get("min"), t.get("max")
                pos_s = f"{pos if pos is not None else '':>5} / {('0x%04X' % pos) if pos is not None else ''}"
                mark = ">" if sid == selected else " "
                stdscr.addstr(
                    row,
                    0,
                    f"{mark}{sid:>2} | {pos_s:>16} | {str(spd or ''):>8} | {str(load or ''):>8} | {str(volt or ''):>6} | {str(temp or ''):>6} | {'' if mn is None else mn}..{'' if mx is None else mx}",
                )
                row += 1
                if row >= h - 14:
                    break

            row += 1
            stdscr.addstr(row, 0, "LOG (last 12):")
            row += 1
            max_lines = min(12, h - row - (len(HELP_FULL) + 2 if show_help[0] else 2))
            for i in range(max_lines):
                tx = tx_log[i] if i < len(tx_log) else ""
                stdscr.addstr(row + i, 0, tx[: w - 1].ljust(w - 1))

            if show_help[0]:
                box_top = row + max_lines + 1
                if box_top < h - 2:
                    stdscr.addstr(box_top, 0, "-" * (w - 1))
                    for i, line in enumerate(HELP_FULL, start=1):
                        if box_top + i >= h - 1:
                            break
                        stdscr.addstr(box_top + i, 0, line[: w - 1])

            stdscr.refresh()

            try:
                ch = stdscr.getch()
            except Exception:
                ch = -1
            if ch == -1:
                time.sleep(0.02)
                continue

            import curses
            if ch in (ord("q"), ord("Q")):
                break
            elif ch in (ord("h"), ord("H")):
                show_help[0] = not show_help[0]
            elif ch == ord(" "):
                paused[0] = not paused[0]
            elif ch == ord("["):
                poll_ms = max(POLL_MS_MIN, poll_ms - 50)
            elif ch == ord("]"):
                poll_ms = min(POLL_MS_MAX, poll_ms + 50)
            elif ch in tuple(ord(str(d)) for d in range(1, 10)):
                selected = int(chr(ch))
            elif ch in (curses.KEY_UP, 259, curses.KEY_DOWN, 258):
                if ids_snapshot:
                    idx = ids_snapshot.index(selected) if selected in ids_snapshot else 0
                    selected = ids_snapshot[(idx + (1 if ch in (curses.KEY_DOWN, 258) else -1)) % len(ids_snapshot)]
            elif ch in (curses.KEY_LEFT, 260):
                if bus.is_open: put(("move", selected, -step_deg_value))
            elif ch in (curses.KEY_RIGHT, 261):
                if bus.is_open: put(("move", selected, +step_deg_value))
            elif ch in (ord("t"), ord("T")):
                if bus.is_open: put(("toggle_torque", selected))
            elif ch in (ord("c"), ord("C")):
                if bus.is_open: put(("center", selected))
            elif ch == ord("R"):
                put(("reconnect",))
            elif ch in (ord("z"), ord("Z"), ord("x"), ord("X")):
                plist = list_serial_ports_sorted(desc=True) or [bus.port]
                idx = (plist.index(bus.port) if bus.port in plist else 0) + (1 if ch in (ord('z'), ord('Z')) else -1)
                put(("switch_port", plist[idx % len(plist)]))

        stop["f"] = True
        bus.close()

    import curses
    curses.wrapper(draw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--model", default="sts3215", help="Feetech model key (e.g. 'sts3215' or 'scs_series')")
    ap.add_argument("--list-ports", action="store_true", help="List detected serial ports and exit")
    ap.add_argument("--probe", action="store_true", help="Probe IDs non-interactively using Feetech SDK, then exit")
    ap.add_argument("--ids", default="1-31", help="IDs to probe, e.g. '1-8' or '1,2,5'")
    ap.add_argument("--ui-ids", default="1-6", help="IDs to track in UI without pinging, e.g. '1-6' or '1,2,5'")
    ap.add_argument("--discover", action="store_true", help="One-shot quick scan at startup to populate UI IDs (uses --ids range)")
    args = ap.parse_args()

    def filtered_ports():
        ps = list_serial_ports_sorted(desc=True)
        if platform.system().lower()=="linux":
            ps = [p for p in ps if p.startswith(("/dev/ttyACM","/dev/ttyUSB"))]
        return ps

    if args.list_ports:
        ports = filtered_ports()
        print("No serial ports detected." if not ports else "Detected ports (best first):\n"+"\n".join(f" - {p}" for p in ports))
        return

    if args.probe:
        ids = parse_ids_arg(args.ids)
        port = best_port_from_list(filtered_ports()) or args.port
        print(f"Probing on port: {port} @ {args.baud} (model={args.model})")
        bus = FeetechBus(port, args.baud, model=args.model)
        try:
            bus.open()
        except Exception as e:
            print(f"Failed to open port: {e}")
            return
        found = []
        for sid in ids:
            ok = bus.ping(sid)
            print(f"PING S{sid}: {'OK' if ok else 'NO'}")
            if ok:
                found.append(sid)
        bus.close()
        print(f"Found IDs: {found if found else 'None'}")
        return

    # Run curses UI
    ui_ids = parse_ids_arg(args.ui_ids)
    discover_ids = parse_ids_arg(args.ids) if args.discover else []
    try:
        run_ui(args.port, args.baud, args.model, ui_ids, discover_ids)
    except ModuleNotFoundError as e:
        print("Feetech SDK not available. Ensure scservo_sdk is installed and PYTHONPATH includes the repo.")
        print(f"Details: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
