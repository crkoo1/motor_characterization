#!/usr/bin/env python3
"""Characterize a solenoid by striking a force sensor (FSR) on A0.

Wiring:  Nano 5V -- FSR -- A0 -- 1 kOhm -- GND
Stop the tuner first: only one program can hold the board's port.

  python3 firmware/characterize/characterize.py check
        read the sensor at rest (should be near 0; press it and run again)
  python3 firmware/characterize/characterize.py hit 21 20
        one hit: note 21, 20 ms of full power, print what the sensor saw
  python3 firmware/characterize/characterize.py calibrate
        rest known weights on the sensor to turn readings into kg
  python3 firmware/characterize/characterize.py sweep 21 --distance-mm 8
        fire note 21 at 1..40 ms, 3 times each, save a CSV and plots
  python3 firmware/characterize/characterize.py live 22 --label solenoid2
        strike when YOU press Enter (for fixtures you reset by hand)
  python3 firmware/characterize/characterize.py sweep 22 --manual --from 10 --to 150 --step 10
        a sweep that waits for Enter before each strike
  python3 firmware/characterize/characterize.py clean results/A.csv
        drop rattles (strikes in contact over 10 ms), save A_clean.csv and plot it
  python3 firmware/characterize/characterize.py plot results/A.csv results/B.csv
        plot one or more sweeps on the same axes (e.g. solenoid 1 vs 2)

Results go in firmware/characterize/results/. The firmware does the timing,
so USB delays don't affect the numbers.
"""
import argparse
import csv
import datetime
import json
import os
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")
CALIBRATION = os.path.join(HERE, "fsr_calibration.json")
G = 9.81


# ---- talking to the board ----------------------------------------------------
def find_port():
    """The Nano shows up as /dev/ttyACM0 on Linux, /dev/cu.usbmodem* on macOS."""
    import glob
    import platform
    if platform.system() == "Darwin":
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    else:
        ports = sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        hint = "/dev/cu.usbmodem*" if platform.system() == "Darwin" else "/dev/ttyACM*"
        sys.exit(f"No board found (no {hint}): is the Nano plugged in?")
    return ports[0]


class Board:
    def __init__(self, port):
        port = port or find_port()
        try:
            self.s = serial.Serial(port, 115200, timeout=0.05, exclusive=True)
        except serial.SerialException as e:
            if "lock" in str(e):
                sys.exit("The board's port is busy. Stop the tuner (Ctrl+C in its tab) and try again.")
            if "Permission" in str(e):
                sys.exit(f"No permission for {port}. Run: sudo setfacl -m u:$USER:rw {port}  "
                         "(or log out and back in once, which fixes it for good)")
            sys.exit(f"Can't open {port}: {e}. Is the Nano plugged in?")
        seen, end = "", time.monotonic() + 2.5          # opening the port may reset the board
        while "READY" not in seen and time.monotonic() < end:
            seen += self.s.read(4096).decode(errors="replace")
        self.s.read(4096)

    def ask(self, command, want, timeout=3.0):
        """Send a command and return the first reply line starting with `want`."""
        self.s.write((command + "\n").encode())
        buf, end = "", time.monotonic() + timeout
        while time.monotonic() < end:
            buf += self.s.read(8192).decode(errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line.startswith(want):
                    self.pending = buf
                    return line
                if line.startswith("ERR"):
                    raise RuntimeError(f"board said: {line}")
        raise RuntimeError(f"no {want} reply to {command!r} (is piano_firmware uploaded?)")

    def test(self, note, ms, duty=4095):
        r = self.ask(f"TESTUS {note} {round(ms * 1000)} {duty}", "RESULT").split()
        # the TRACE line follows right after RESULT
        buf, end = self.pending, time.monotonic() + 3
        while "\n" not in buf and time.monotonic() < end:
            buf += self.s.read(8192).decode(errors="replace")
        t = buf.split("\n", 1)[0].split()
        if not t or t[0] != "TRACE":
            raise RuntimeError("missing TRACE line")
        names = ["note", "pulse_us", "duty", "rest", "contact_us", "peak", "peak_us", "width_us", "area",
                 "on_us", "i2c_us"]
        result = dict(zip(names, map(int, r[1:])))
        if "on_us" not in result:
            raise RuntimeError("old firmware on the board: upload piano_firmware again")
        result["ms"] = result["pulse_us"] / 1000
        result["sample_us"] = int(t[1])
        result["trace"] = list(map(int, t[2:]))
        return result

    def fsr(self):
        _, avg, lo, hi = self.ask("FSR", "FSR").split()
        return int(avg), int(lo), int(hi)

    def idle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.s.write(b"PING\n")
            time.sleep(min(0.1, max(0, end - time.monotonic())))
        self.s.read(8192)

    def close(self):
        try:                                   # the board may already be unplugged
            self.s.write(b"ALLOFF\n")
            time.sleep(0.1)
            self.s.close()
        except (OSError, serial.SerialException):
            print("(board already disconnected)")


# ---- turning readings into force ---------------------------------------------
def load_calibration():
    if not os.path.exists(CALIBRATION):
        return None
    with open(CALIBRATION) as f:
        return sorted(json.load(f)["points"])          # [[adc, kg], ...]


def to_kg(adc, cal):
    """Piecewise-linear from the calibration points; None without a calibration."""
    if not cal:
        return None
    if adc <= cal[0][0]:
        return cal[0][1]
    for (a0, k0), (a1, k1) in zip(cal, cal[1:]):
        if adc <= a1:
            return k0 + (adc - a0) * (k1 - k0) / (a1 - a0)
    a0, k0 = cal[-2] if len(cal) > 1 else (0, 0)
    a1, k1 = cal[-1]
    return k1 + (adc - a1) * (k1 - k0) / max(a1 - a0, 1)   # beyond the heaviest weight: a guess


def summarize(r, cal, distance_mm=None):
    contact_ms = r["contact_us"] / 1000 if r["contact_us"] >= 0 else None
    row = {
        "note": r["note"], "ms": r["ms"], "on_ms": r["on_us"] / 1000, "duty": r["duty"], "rest": r["rest"],
        "contact_ms": contact_ms,
        "peak_adc": r["peak"] - r["rest"],
        "peak_ms": r["peak_us"] / 1000,
        "width_ms": r["width_us"] / 1000,
        "area": r["area"],
        "peak_kg": None, "impulse_Ns": None, "speed_m_s": None,
    }
    if cal:
        rest_kg = to_kg(r["rest"], cal)
        row["peak_kg"] = round(to_kg(r["peak"], cal) - rest_kg, 3)
        dt = r["sample_us"] / 1e6
        row["impulse_Ns"] = round(sum(max(0, to_kg(v, cal) - rest_kg) for v in r["trace"]) * G * dt, 5)
    if distance_mm and contact_ms:
        row["speed_m_s"] = round(distance_mm / contact_ms, 3)  # mm/ms = m/s, average from pulse start
    return row


def describe(row):
    if row["contact_ms"] is None:
        return f"{row['ms']:>5g} ms pulse: no contact (didn't reach the sensor, or threshold too high)"
    s = f"{row['ms']:>5g} ms pulse: contact at {row['contact_ms']:6.2f} ms, peak {row['peak_adc']:4d} counts"
    if row["peak_kg"] is not None:
        s += f" ({row['peak_kg']:.2f} kg)"
    s += f" at {row['peak_ms']:.2f} ms, in contact {row['width_ms']:.1f} ms"
    if row["speed_m_s"]:
        s += f", avg speed {row['speed_m_s']:.2f} m/s"
    return s


def sparkline(trace, rest, width=75):
    bars = " ▁▂▃▄▅▆▇█"
    step = max(1, len(trace) // width)
    cols = [max(trace[i:i + step]) - rest for i in range(0, len(trace), step)]
    top = max(max(cols), 1)
    return "".join(bars[max(0, min(8, round(c / top * 8)))] for c in cols)


# ---- commands ---------------------------------------------------------------
def cmd_check(a):
    b = Board(a.port)
    try:
        avg, lo, hi = b.fsr()
        print(f"Sensor reads {avg} (min {lo}, max {hi}) out of 1023.")
        if avg > 50:
            print("Something is pressing on it. If nothing should be, check nothing rests on the disc.")
        elif hi - lo > 15:
            print("It's noisy: check the A0 and resistor connections.")
        else:
            print("Looks good. Press the sensor with a finger and run this again: the number should jump.")
    finally:
        b.close()


def cmd_hit(a):
    b, cal = Board(a.port), load_calibration()
    try:
        r = b.test(a.note, a.ms, a.duty)
        row = summarize(r, cal, a.distance_mm)
        print(describe(row))
        print(f"0 ms {sparkline(r['trace'], r['rest'])} {len(r['trace']) * r['sample_us'] // 1000} ms")
    finally:
        b.close()


def cmd_calibrate(a):
    b = Board(a.port)
    points = []
    try:
        input("Take everything off the sensor (keep the disc on it if you use one), then press Enter...")
        avg, _, _ = b.fsr()
        points.append([avg, 0.0])
        print(f"  nothing: {avg}")
        while True:
            kg = input("Rest a known weight on the disc, type its mass in kg (blank = done): ").strip()
            if not kg:
                break
            avg, lo, hi = b.fsr()
            points.append([avg, float(kg)])
            print(f"  {kg} kg: {avg} (min {lo}, max {hi})")
            if avg > 1000:
                print("  That's near the top of the range: use a smaller resistor (e.g. 470 ohm) for heavier forces.")
    finally:
        b.close()
    points.sort()
    if len(points) < 3:
        print("Not saved: use at least two weights so we can draw a curve.")
        return
    with open(CALIBRATION, "w") as f:
        json.dump({"date": datetime.date.today().isoformat(), "resistor_ohm": a.resistor,
                   "points": points}, f, indent=2)
    print(f"Saved {len(points)} points to {os.path.relpath(CALIBRATION)}")


def cmd_sweep(a):
    os.makedirs(RESULTS, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    name = f"{stamp}_note{a.note}" + (f"_{a.label}" if a.label else "")
    csv_path = os.path.join(RESULTS, name + ".csv")
    n = int(round((a.stop - a.start) / a.step))
    pulses = [round(a.start + i * a.step, 3) for i in range(n + 1)]
    cal = load_calibration()
    if not cal:
        print("No calibration yet: force will be in raw sensor counts, not kg.")
    print(f"Note {a.note}: pulses {pulses[0]}-{pulses[-1]} ms, {a.repeats} each, "
          f"{a.wait} s apart, about {len(pulses) * a.repeats * (a.wait + 0.6) / 60:.1f} min. Ctrl+C stops.")

    if a.manual:
        print("Manual: reset the fixture before each strike. Enter = strike, "
              "r = redo the last strike, s = skip, q = stop and save.")
    b = Board(a.port)
    rows, traces = [], []
    try:
        steps = [(rep, ms) for rep in range(a.repeats) for ms in pulses]  # repeats outermost: heat drift spreads out
        i = 0
        while i < len(steps):
            rep, ms = steps[i]
            if a.manual:
                key = input(f"[{i + 1}/{len(steps)}] next: {ms} ms. Reset fixture, Enter to strike: ").strip().lower()
                if key == "q":
                    break
                if key == "s":
                    i += 1
                    continue
                if key == "r" and rows:
                    rows.pop()
                    traces.pop()
                    i -= 1
                    rep, ms = steps[i]
                    print(f"Redoing {ms} ms.")
            r = b.test(a.note, ms, a.duty)
            row = summarize(r, cal, a.distance_mm)
            row["repeat"] = rep + 1
            rows.append(row)
            traces.append({"ms": ms, "repeat": rep + 1, "rest": r["rest"],
                           "sample_us": r["sample_us"], "trace": r["trace"]})
            print(f"[{rep + 1}/{a.repeats}] {describe(row)}")
            i += 1
            if not a.manual:
                b.idle(a.wait)
    except (OSError, serial.SerialException):
        print("\nLost the board (unplugged?); saving what we have.")
    except (KeyboardInterrupt, EOFError):
        print("\nStopped early; saving what we have.")
    finally:
        b.close()
    save(rows, traces, csv_path)


def cmd_live(a):
    """Strike on demand: type a pulse length, Enter repeats the last one."""
    os.makedirs(RESULTS, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = os.path.join(RESULTS, f"{stamp}_note{a.note}" + (f"_{a.label}" if a.label else "") + ".csv")
    cal = load_calibration()
    print(f"Live, note {a.note}. Reset the fixture, then type a pulse length in ms and press Enter.\n"
          "Enter alone repeats the last length. d = delete the last strike (bad reset). q = quit and save.")
    b = Board(a.port)
    rows, traces, ms, counts = [], [], a.ms, {}
    try:
        while True:
            key = input(f"pulse ms [{ms}]: ").strip().lower()
            if key == "q":
                break
            if key == "d":
                if rows:
                    gone = rows.pop()
                    traces.pop()
                    counts[gone["ms"]] -= 1
                    print(f"Deleted the last {gone['ms']} ms strike.")
                continue
            if key:
                try:
                    ms = float(key)
                except ValueError:
                    print("Type a number of ms, Enter, d or q.")
                    continue
            try:
                r = b.test(a.note, ms, a.duty)
            except RuntimeError as e:
                print(e)
                continue
            counts[ms] = counts.get(ms, 0) + 1
            row = summarize(r, cal, a.distance_mm)
            row["repeat"] = counts[ms]
            rows.append(row)
            traces.append({"ms": ms, "repeat": counts[ms], "rest": r["rest"],
                           "sample_us": r["sample_us"], "trace": r["trace"]})
            print(describe(row))
            print(f"   0 ms {sparkline(r['trace'], r['rest'], 60)} 150 ms")
    except (OSError, serial.SerialException):
        print("\nLost the board (unplugged?); saving what we have.")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        b.close()
    save(rows, traces, csv_path)


def save(rows, traces, csv_path):
    if not rows:
        print("Nothing to save.")
        return
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(csv_path.removesuffix(".csv") + "_traces.json", "w") as f:
        json.dump(traces, f)
    print(f"Saved {os.path.relpath(csv_path)}")
    plot([csv_path])


def cmd_clean(a):
    """Copy a run without its rattles: strikes in contact longer than --max-width-ms."""
    src = a.csv
    with open(src) as f:
        rows = list(csv.DictReader(f))
    tpath = src.removesuffix(".csv") + "_traces.json"
    traces = json.load(open(tpath)) if os.path.exists(tpath) else None
    keep, dropped = [], []
    for i, r in enumerate(rows):
        (dropped if float(r["width_ms"]) > a.max_width_ms else keep).append(i)
    out = src.removesuffix(".csv") + "_clean.csv"
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows[i] for i in keep)
    if traces and len(traces) == len(rows):
        with open(out.removesuffix(".csv") + "_traces.json", "w") as f:
            json.dump([traces[i] for i in keep], f)
    print(f"Kept {len(keep)} of {len(rows)} strikes; dropped {len(dropped)} in contact over {a.max_width_ms:g} ms:")
    for i in dropped:
        print(f"   {rows[i]['ms']:>4} ms pulse, repeat {rows[i]['repeat']}: in contact {float(rows[i]['width_ms']):g} ms")
    print(f"Saved {os.path.relpath(out)}  (the original is unchanged)")
    plot([out])


def plot(paths):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    (ax_contact, ax_peak), (ax_impulse, ax_trace) = axes
    for path in paths:
        with open(path) as f:
            rows = list(csv.DictReader(f))
        label = os.path.basename(path).removesuffix(".csv")
        use_kg = rows and rows[0]["peak_kg"] not in ("", "None")
        by_ms = {}
        for r in rows:
            by_ms.setdefault(float(r["ms"]), []).append(r)
        xs = sorted(by_ms)

        def series(key):
            pts = [(x, [float(r[key]) for r in by_ms[x] if r[key] not in ("", "None")]) for x in xs]
            pts = [(x, v) for x, v in pts if v]
            return ([x for x, _ in pts], [sum(v) / len(v) for _, v in pts],
                    [min(v) for _, v in pts], [max(v) for _, v in pts])

        for ax, key in [(ax_contact, "contact_ms"), (ax_peak, "peak_kg" if use_kg else "peak_adc"),
                        (ax_impulse, "impulse_Ns" if use_kg else "area")]:
            x, mean, lo, hi = series(key)
            if x:
                line, = ax.plot(x, mean, marker="o", ms=3, label=label)
                ax.fill_between(x, lo, hi, alpha=0.2, color=line.get_color())

        tpath = path.removesuffix(".csv") + "_traces.json"
        if os.path.exists(tpath):
            with open(tpath) as f:
                tr = [t for t in json.load(f) if t["repeat"] == 1]
            for t in tr[:: max(1, len(tr) // 6)]:
                ts = [i * t["sample_us"] / 1000 for i in range(len(t["trace"]))]
                ax_trace.plot(ts, [v - t["rest"] for v in t["trace"]], lw=1, label=f"{label} {t['ms']} ms")

    ax_contact.set(title="Time to hit the sensor (lower = faster)", xlabel="pulse length (ms)", ylabel="ms from pulse start")
    ax_peak.set(title="Peak force", xlabel="pulse length (ms)", ylabel="kg" if use_kg else "sensor counts above rest")
    ax_impulse.set(title="Impulse (how much push the hit delivered)", xlabel="pulse length (ms)",
                   ylabel="N·s" if use_kg else "area (counts × samples)")
    ax_trace.set(title="Sensor during a few hits", xlabel="ms from pulse start", ylabel="counts above rest")
    for ax in axes.flat:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    out = paths[0].removesuffix(".csv") + (".png" if len(paths) == 1 else "_compare.png")
    fig.savefig(out, dpi=110)
    print(f"Plot: {os.path.relpath(out)}")


def main():
    p = argparse.ArgumentParser(description="Characterize solenoids with a force sensor.")
    p.add_argument("--port", help="serial port (default: the first /dev/ttyACM*)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="read the sensor at rest")

    h = sub.add_parser("hit", help="one test hit")
    h.add_argument("note", type=int)
    h.add_argument("ms", type=float)
    h.add_argument("--duty", type=int, default=4095, help="power 1-4095 (below 4095 is PWM: fit a flyback diode first)")
    h.add_argument("--distance-mm", type=float, help="plunger-to-sensor gap, to estimate speed")

    c = sub.add_parser("calibrate", help="calibrate the sensor with known weights")
    c.add_argument("--resistor", type=int, default=1000, help="divider resistor in ohms, for the record")

    s = sub.add_parser("sweep", help="fire at a range of pulse lengths and plot")
    s.add_argument("note", type=int)
    s.add_argument("--from", dest="start", type=float, default=1, help="shortest pulse, ms (0.5 steps are fine)")
    s.add_argument("--to", dest="stop", type=float, default=40)
    s.add_argument("--step", type=float, default=1)
    s.add_argument("--repeats", type=int, default=3)
    s.add_argument("--wait", type=float, default=1.0, help="seconds between hits (lets the coil cool)")
    s.add_argument("--duty", type=int, default=4095, help="power 1-4095 (below 4095 is PWM: fit a flyback diode first)")
    s.add_argument("--distance-mm", type=float, help="plunger-to-sensor gap, to estimate speed")
    s.add_argument("--label", help="added to the file name, e.g. solenoid1")
    s.add_argument("--manual", action="store_true", help="wait for Enter before each strike (fixture reset)")

    lv = sub.add_parser("live", help="strike on demand, one Enter at a time")
    lv.add_argument("note", type=int)
    lv.add_argument("--ms", type=float, default=40, help="starting pulse length")
    lv.add_argument("--duty", type=int, default=4095, help="power 1-4095 (below 4095 is PWM: fit a flyback diode first)")
    lv.add_argument("--distance-mm", type=float, help="plunger-to-sensor gap, to estimate speed")
    lv.add_argument("--label", help="added to the file name, e.g. solenoid2")

    cl = sub.add_parser("clean", help="copy a run without rattling strikes, then plot it")
    cl.add_argument("csv")
    cl.add_argument("--max-width-ms", type=float, default=10, help="drop strikes in contact longer than this")

    pl = sub.add_parser("plot", help="plot saved sweeps together")
    pl.add_argument("csv", nargs="+")

    a = p.parse_args()
    if getattr(a, "duty", 4095) < 4095:
        print("Note: duty below 4095 switches the coil ~1000 times a second. Without a flyback "
              "diode across the solenoid, that can kill the MOSFET.")
    {"check": cmd_check, "hit": cmd_hit, "calibrate": cmd_calibrate,
     "sweep": cmd_sweep, "live": cmd_live, "clean": cmd_clean, "plot": lambda a: plot(a.csv)}[a.cmd](a)


if __name__ == "__main__":
    main()
