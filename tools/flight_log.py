#!/usr/bin/env python3
"""
flight_log.py - download, summarise and plot the flight logs of the ESP32-S3 drone.

The flight controller (firmware/4_flight_controller) records 50 lines per second
while armed and saves them to its flash after landing. This tool talks to it over
USB, exactly like the Arduino Serial Monitor would (close the Serial Monitor first).

  python flight_log.py ports                        list the serial ports
  python flight_log.py list     --port COM10        list the logs saved on the drone
  python flight_log.py download --port COM10        newest log  -> flight_YYYYmmdd_HHMMSS.csv
  python flight_log.py download --port COM10 --log 3 --out test3.csv
  python flight_log.py summary  test3.csv           text summary: time, height, battery, events
  python flight_log.py plot     test3.csv           graphs (add --save test3.png to write a file)

Install once:  pip install -r requirements.txt   (pyserial, matplotlib, numpy)
The drone must be DISARMED (SwA on SAFE). USB only is fine: the logs are in flash.
"""
import argparse
import csv
import os
import sys
import time
from datetime import datetime

MODES = {0: "MANUAL", 1: "ALT HOLD", 2: "LAND"}
MODE_COLORS = {1: "#e6daf5", 2: "#ffe2bf"}          # ALT HOLD purple, LAND orange
BAUD = 115200
CELL_WARN_V, CELL_CRITICAL_V = 3.75, 3.65           # same as config.h (for the plot lines)


# --------------------------------------------------------------------------- serial
def open_port(port):
    import serial
    s = serial.Serial()
    s.port = port
    s.baudrate = BAUD
    s.timeout = 0.2
    # Keep DTR/RTS low: on the DevKit's "COM" port they are wired to EN/BOOT, and
    # toggling them would reset the ESP32 (then it needs ~12 s to start again).
    s.dtr = False
    s.rts = False
    s.open()
    return s


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    known = [p for p in ports if p.vid in (0x303A, 0x1A86, 0x10C4, 0x0403)]   # Espressif, CH34x, CP210x, FTDI
    pick = known or ports
    if len(pick) == 1:
        return pick[0].device
    print("Please choose the port with --port. Found:")
    for p in ports:
        print(f"  {p.device:12s} {p.description}")
    sys.exit(1)


def send(s, cmd):
    s.write((cmd + "\n").encode())
    s.flush()


def read_lines(s, seconds):
    """Yields text lines for 'seconds' seconds."""
    end = time.time() + seconds
    buf = b""
    while time.time() < end:
        buf += s.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield line.decode(errors="replace").rstrip("\r")


def cmd_ports(_args):
    from serial.tools import list_ports
    for p in list_ports.comports():
        vid = f"{p.vid:04X}" if p.vid is not None else "----"
        print(f"{p.device:14s} VID {vid}  {p.description}")


def cmd_list(args):
    s = open_port(args.port or find_port())
    send(s, "l")
    got = False
    for line in read_lines(s, 3.0):
        if line.startswith(("Saved flight logs", "  log_", "  (none)", "Flash:", "No flash")):
            print(line)
            got = True
    if not got:
        print("No answer. Is the drone DISARMED, is the Serial Monitor closed, is it the right port?")


def cmd_download(args):
    s = open_port(args.port or find_port())
    cmd = "d" if args.log is None else f"d{args.log}"
    rows, header, started, deadline = [], None, False, time.time() + args.timeout
    next_send = 0.0
    print(f"Asking the drone for {'the newest log' if args.log is None else 'log ' + str(args.log)} ...")
    while time.time() < deadline:
        if not started and time.time() >= next_send:      # (re)send until the dump starts
            send(s, cmd)
            next_send = time.time() + 4.0
        for line in read_lines(s, 0.5):
            if line.startswith("BEGIN_LOG"):
                started, rows, header = True, [], None
                print(line)
                continue
            if not started:
                if line.startswith(("Cannot read", "No saved log")) or "another firmware" in line:
                    print("Drone says:", line)
                    return 1
                continue
            if line.startswith("END_OF_LOG"):
                return save_csv(header, rows, args.out)
            if header is None:
                header = line
            elif line and line[0].isdigit():
                rows.append(line)
            if len(rows) % 2500 == 0 and rows:
                print(f"  {len(rows)} lines...")
    print("Timed out. Is the drone DISARMED and the Serial Monitor closed?")
    return 1


def save_csv(header, rows, out):
    if header is None:
        print("Empty log.")
        return 1
    out = out or datetime.now().strftime("flight_%Y%m%d_%H%M%S.csv")
    with open(out, "w", newline="") as f:
        f.write(header + "\n")
        for r in rows:
            f.write(r + "\n")
    print(f"Saved {len(rows)} lines ({len(rows) / 50.0:.1f} s of flight) to {out}")
    return 0


# --------------------------------------------------------------------------- analysis
def load(path):
    with open(path, newline="") as f:
        rd = csv.DictReader(line for line in f if not line.startswith(("BEGIN_LOG", "END_OF_LOG")))
        data = {k: [] for k in rd.fieldnames}
        for row in rd:
            for k in rd.fieldnames:
                try:
                    data[k].append(float(row[k]))
                except (TypeError, ValueError):
                    data[k].append(float("nan"))
    if not data.get("t_s"):
        sys.exit(f"{path}: no data")
    return data


def events(d):
    """Mode changes and warnings, as (time, text)."""
    ev, t = [], d["t_s"]
    prev = {"mode": int(d["mode"][0])}
    for i in range(len(t)):
        mode = int(d["mode"][i])
        if prev.get("mode") != mode:
            why = ""
            if mode == 2:
                why = " (radio lost)" if d["land_radio"][i] else " (battery)" if d["land_batt"][i] else ""
            ev.append((t[i], f"mode {MODES.get(mode, mode)}{why}"))
        for key, on_txt, off_txt in (("airborne", "airborne", "on the ground"),
                                     ("radio_ok", "radio back", "RADIO LOST"),
                                     ("batt_low", "battery LOW", None),
                                     ("batt_crit", "battery CRITICAL", None),
                                     ("baro_ok", None, "BAROMETER LOST")):
            v = int(d[key][i])
            if key in prev and v != prev[key]:
                txt = on_txt if v else off_txt
                if txt:
                    ev.append((t[i], txt))
            prev[key] = v
        prev["mode"] = mode
    return ev


def cmd_summary(args):
    d = load(args.csv)
    t = d["t_s"]
    n = len(t)
    dur = t[-1] - t[0]
    air = sum(1 for v in d["airborne"] if v) / 50.0
    print(f"File ............ {args.csv}")
    print(f"Armed ........... {dur:.1f} s ({n} lines), in the air about {air:.1f} s")
    print(f"Highest ......... {max(d['alt']):.2f} m (fused height above take-off)")
    for m, name in MODES.items():
        secs = sum(1 for v in d["mode"] if int(v) == m) / 50.0
        if secs:
            print(f"Time in {name:9s} {secs:6.1f} s")
    print(f"Battery ......... {d['vbat'][0]:.2f} V -> {d['vbat'][-1]:.2f} V measured, "
          f"lowest {min(d['vbat']):.2f} V under load")
    if "vrest" in d:
        print(f"Charge .......... {int(d['batt_pct'][0])} % -> {int(d['batt_pct'][-1])} % "
              f"(estimated resting {d['vrest'][-1]:.2f} V)")
    sat = sum(1 for i in range(n) for m in ("m1", "m2", "m3", "m4") if d[m][i] >= 1995) / (4.0 * n) * 100
    print(f"Motors at 100 % . {sat:.1f} % of the time (if high: the drone lacks power or the PIDs fight)")
    tilt = max(max(abs(v) for v in d["roll"]), max(abs(v) for v in d["pitch"]))
    print(f"Max tilt ........ {tilt:.1f} deg")
    print("Events:")
    for te, txt in events(d):
        print(f"  {te - t[0]:7.2f} s  {txt}")


def shade_modes(ax, d):
    t, mode = d["t_s"], d["mode"]
    start = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or mode[i] != mode[start]:
            c = MODE_COLORS.get(int(mode[start]))
            if c:
                ax.axvspan(t[start], t[i - 1], color=c, lw=0, zorder=0)
            start = i


def cmd_plot(args):
    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    d = load(args.csv)
    t = [v - d["t_s"][0] for v in d["t_s"]]
    d["t_s"] = t
    fig, ax = plt.subplots(6, 1, figsize=(11, 15), sharex=True,
                           gridspec_kw={"height_ratios": [3, 2, 2, 2, 2, 1.3]})
    fig.suptitle(f"Flight log  {os.path.basename(args.csv)}", fontsize=12)

    a = ax[0]
    a.plot(t, d["baro_alt"], color="#bbbbbb", lw=0.8, label="barometer only")
    a.plot(t, d["alt"], color="#1f5fae", lw=1.6, label="height (fused)")
    alt_mode = [d["alt_target"][i] if int(d["mode"][i]) == 1 else float("nan") for i in range(len(t))]
    a.plot(t, alt_mode, "--", color="#7b3fb5", lw=1.2, label="ALT HOLD target")
    a.set_ylabel("height (m)")
    a.legend(loc="upper left", fontsize=8)

    a = ax[1]
    a.plot(t, d["vz"], color="#1f5fae", lw=1.2)
    a.axhline(0, color="#888", lw=0.6)
    a.set_ylabel("vertical speed (m/s)")

    a = ax[2]
    a.plot(t, d["roll"], color="#c0392b", lw=1.0, label="roll")
    a.plot(t, d["roll_sp"], "--", color="#c0392b", lw=0.8, alpha=0.7, label="roll wanted")
    a.plot(t, d["pitch"], color="#27ae60", lw=1.0, label="pitch")
    a.plot(t, d["pitch_sp"], "--", color="#27ae60", lw=0.8, alpha=0.7, label="pitch wanted")
    a.set_ylabel("angle (deg)")
    a.legend(loc="upper left", fontsize=8, ncol=4)

    a = ax[3]
    for m, c in zip(("m1", "m2", "m3", "m4"), ("#e67e22", "#16a085", "#8e44ad", "#7f8c8d")):
        a.plot(t, d[m], lw=0.6, color=c, alpha=0.8, label=m.upper())
    a.plot(t, d["thr_out"], color="black", lw=1.4, label="throttle used")
    a.plot(t, d["thr_in"], ":", color="black", lw=1.0, label="throttle stick")
    a.set_ylabel("signal (us)")
    a.legend(loc="upper left", fontsize=8, ncol=6)

    a = ax[4]
    cells = args.cells or max(1, round(max(d.get("vrest", d["vbat"])) / 3.85))
    a.plot(t, [v / cells for v in d["vbat"]], color="#c0392b", lw=1.0, label="measured (sags under load)")
    if "vrest" in d:
        a.plot(t, [v / cells for v in d["vrest"]], color="#1f5fae", lw=1.4, label="estimated resting")
        a.axhline(CELL_WARN_V, color="#d4a017", lw=0.8, ls="--", label=f"LOW {CELL_WARN_V} V")
        a.axhline(CELL_CRITICAL_V, color="#c0392b", lw=0.8, ls="--", label=f"CRITICAL {CELL_CRITICAL_V} V")
    a.set_ylabel(f"battery V/cell ({cells}S)")
    a.legend(loc="lower left", fontsize=8, ncol=2)

    a = ax[5]
    if "batt_pct" in d:
        a.plot(t, d["batt_pct"], color="#27ae60", lw=1.4)
        a.set_ylim(0, 105)
    a.set_ylabel("charge (%)")
    a.set_xlabel("time since arming (s)")

    for a in ax:
        shade_modes(a, d)
        a.grid(alpha=0.3)
    merged = []                                    # events closer than 0.3 s share one label
    for te, txt in events(d):
        if merged and te - merged[-1][0] < 0.3:
            merged[-1] = (merged[-1][0], merged[-1][1] + " + " + txt)
        else:
            merged.append((te, txt))
    for te, txt in merged:
        for a in ax:
            a.axvline(te, color="#555", lw=0.6, ls=":")
        ax[0].annotate(txt, (te, 0.03), xycoords=("data", "axes fraction"), rotation=90,
                       fontsize=7, va="bottom", ha="right", color="#333")
    ax[0].legend(handles=ax[0].get_legend_handles_labels()[0] +
                 [Patch(color=MODE_COLORS[1], label="ALT HOLD"), Patch(color=MODE_COLORS[2], label="LAND")],
                 loc="upper right", fontsize=8, ncol=2)
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=110)
        print(f"Saved {args.save}")
    else:
        plt.show()


# --------------------------------------------------------------------------- main
def main():
    p = argparse.ArgumentParser(description="Download, summarise and plot drone flight logs.",
                                formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="list serial ports").set_defaults(fn=cmd_ports)
    q = sub.add_parser("list", help="list the logs saved on the drone")
    q.add_argument("--port")
    q.set_defaults(fn=cmd_list)
    q = sub.add_parser("download", help="download a log as CSV")
    q.add_argument("--port")
    q.add_argument("--log", type=int, help="log number (default: newest)")
    q.add_argument("--out", help="CSV file name")
    q.add_argument("--timeout", type=float, default=60.0, help="seconds to wait (default 60)")
    q.set_defaults(fn=cmd_download)
    q = sub.add_parser("summary", help="print a text summary of a CSV log")
    q.add_argument("csv")
    q.set_defaults(fn=cmd_summary)
    q = sub.add_parser("plot", help="plot a CSV log")
    q.add_argument("csv")
    q.add_argument("--cells", type=int, help="LiPo cells (default: guessed from the voltage)")
    q.add_argument("--save", help="write the figure to this PNG instead of showing it")
    q.set_defaults(fn=cmd_plot)
    a = p.parse_args()
    sys.exit(a.fn(a) or 0)


if __name__ == "__main__":
    main()
