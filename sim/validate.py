"""
Monte-Carlo validation of the arc-fault detector.

Runs N randomized trials of every event class through the reference
detector, reports per-class detection / nuisance-trip rates and trip-time
statistics, sweeps the persistence threshold to expose the sensitivity/
nuisance tradeoff, and writes example plots to docs/img/.

Usage:  python3 sim/validate.py [--trials N] [--seed S]
"""

from __future__ import annotations

import argparse
import os

import numpy as np

import waveforms as wf
import detector as det

IMG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "img")


def run_trials(n_trials: int) -> dict:
    results = {}
    for gen in wf.ALL_GENERATORS:
        trips, times = 0, []
        for _ in range(n_trials):
            tr = gen()
            st, _ = det.process_trace(tr.hf_env, tr.i_lf)
            if st.tripped:
                trips += 1
                tt = det.trip_time_s(st, tr.arc_onset_s)
                if tt is not None:
                    times.append(tt)
        results[gen.__name__] = {
            "label": gen().label,
            "trips": trips,
            "n": n_trials,
            "times": np.array(times),
        }
    return results


def print_report(results: dict) -> str:
    lines = []
    lines.append(f"{'event class':<20}{'type':<9}{'trip rate':<12}"
                 f"{'median t_trip':<15}{'p95 t_trip':<12}")
    lines.append("-" * 68)
    for name, r in results.items():
        rate = r["trips"] / r["n"]
        if r["label"] == "arc":
            med = f"{np.median(r['times'])*1000:.0f} ms" if len(r["times"]) else "--"
            p95 = f"{np.percentile(r['times'], 95)*1000:.0f} ms" if len(r["times"]) else "--"
        else:
            med = p95 = "--"
        lines.append(f"{name:<20}{r['label']:<9}{rate*100:6.1f} %     {med:<15}{p95:<12}")
    arc_n = sum(r["n"] for r in results.values() if r["label"] == "arc")
    arc_t = sum(r["trips"] for r in results.values() if r["label"] == "arc")
    nrm_n = sum(r["n"] for r in results.values() if r["label"] == "normal")
    nrm_t = sum(r["trips"] for r in results.values() if r["label"] == "normal")
    lines.append("-" * 68)
    lines.append(f"overall arc detection : {arc_t}/{arc_n}  ({arc_t/arc_n*100:.1f} %)")
    lines.append(f"overall nuisance trips: {nrm_t}/{nrm_n}  ({nrm_t/nrm_n*100:.2f} %)")
    text = "\n".join(lines)
    print(text)
    return text


def sweep_persistence(n_trials: int) -> list[tuple[int, float, float]]:
    """Sweep N_SLOW (with N_FAST = N_SLOW//2) to show the tradeoff curve."""
    rows = []
    orig_slow, orig_fast = det.N_SLOW, det.N_FAST
    for n_slow in (4, 6, 8, 10, 12, 14, 18, 24):
        det.N_SLOW, det.N_FAST = n_slow, max(2, n_slow // 2)
        miss, nuis, n_a, n_n = 0, 0, 0, 0
        for gen in wf.ALL_GENERATORS:
            for _ in range(n_trials):
                tr = gen()
                st, _ = det.process_trace(tr.hf_env, tr.i_lf)
                if tr.label == "arc":
                    n_a += 1
                    miss += 0 if st.tripped else 1
                else:
                    n_n += 1
                    nuis += 1 if st.tripped else 0
        rows.append((n_slow, miss / n_a, nuis / n_n))
        print(f"  N_SLOW={n_slow:3d}: missed arcs {miss/n_a*100:5.1f} %   "
              f"nuisance {nuis/n_n*100:5.1f} %")
    det.N_SLOW, det.N_FAST = orig_slow, orig_fast
    return rows


def make_plots(sweep_rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(IMG_DIR, exist_ok=True)
    np.random.seed(7)

    # ---- 1. example traces: what the MCU actually sees ----
    cases = [wf.series_arc(), wf.load_step(), wf.inrush(), wf.pwm_load()]
    fig, axes = plt.subplots(len(cases), 2, figsize=(11, 9), sharex="col")
    for row, tr in enumerate(cases):
        t = tr.t_adc * 1000
        axes[row, 0].plot(t, tr.i_lf, lw=0.6, color="#1668a8")
        axes[row, 0].set_ylabel(f"{tr.name}\nADC counts", fontsize=8)
        axes[row, 1].plot(t, tr.hf_env, lw=0.6, color="#b0413e")
        if tr.arc_onset_s:
            for ax in axes[row]:
                ax.axvline(tr.arc_onset_s * 1000, color="k", ls=":", lw=0.8)
    axes[0, 0].set_title("ch0: LF current")
    axes[0, 1].set_title("ch1: HF noise envelope")
    axes[-1, 0].set_xlabel("time [ms]")
    axes[-1, 1].set_xlabel("time [ms]")
    fig.suptitle("What the ADC sees: arc vs. the three main nuisance events")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG_DIR, "example_waveforms.png"), dpi=130)
    plt.close(fig)

    # ---- 2. detector internals on an arc vs a PWM load ----
    fig, axes = plt.subplots(2, 2, figsize=(11, 6), sharex=True)
    for col, tr in enumerate([wf.series_arc(), wf.pwm_load()]):
        st, log = det.process_trace(tr.hf_env, tr.i_lf, collect=True)
        tw = np.arange(len(log["hf_mean"])) * det.WIN / 4.0  # ms
        axes[0, col].plot(tw, log["hf_mean"], label="hf_mean", lw=0.9)
        axes[0, col].plot(tw, log["baseline"], label="baseline", lw=0.9)
        axes[0, col].plot(tw, log["hf_mad"], label="hf_mad", lw=0.9)
        axes[0, col].set_title(f"{tr.name}  (tripped={st.tripped})")
        axes[0, col].legend(fontsize=7)
        axes[1, col].plot(tw, log["counter"], lw=1.0, color="#7a4f9e")
        axes[1, col].axhline(det.N_SLOW, color="r", ls="--", lw=0.8, label="N_SLOW")
        axes[1, col].axhline(det.N_FAST, color="orange", ls="--", lw=0.8, label="N_FAST")
        if st.tripped:
            axes[1, col].axvline((st.trip_window + 1) * det.WIN / 4.0,
                                 color="r", lw=1.2)
        axes[1, col].set_ylabel("persistence")
        axes[1, col].set_xlabel("time [ms]")
        axes[1, col].legend(fontsize=7)
    fig.suptitle("Detector internals: arc trips, PWM load does not")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG_DIR, "detector_internals.png"), dpi=130)
    plt.close(fig)

    # ---- 3. feature space scatter ----
    fig, ax = plt.subplots(figsize=(7, 5.5))
    colors = {"series_arc": "#b0413e", "parallel_arc": "#e07b39",
              "arc_during_pwm": "#8c2d2a", "load_step_bounce": "#1668a8",
              "inrush": "#2a9d8f", "pwm_load": "#5e60ce",
              "brushed_motor": "#7f8c4f", "steady_load": "#999999"}
    for gen in wf.ALL_GENERATORS:
        xs, ys = [], []
        for _ in range(30):
            tr = gen()
            _, log = det.process_trace(tr.hf_env, tr.i_lf, collect=True)
            elev = log["hf_mean"].astype(int) - log["baseline"].astype(int)
            k = np.argsort(elev)[-8:]  # most-elevated windows of the event
            xs.extend(elev[k])
            ys.extend(log["hf_mad"][k])
        marker = "x" if tr.label == "arc" else "o"
        ax.scatter(xs, ys, s=10, alpha=0.45, marker=marker,
                   label=tr.name, color=colors.get(tr.name, "k"))
    ax.set_xlabel("HF envelope elevation above baseline [counts]  (feature A)")
    ax.set_ylabel("HF envelope MAD within window [counts]  (feature B)")
    ax.set_xlim(-10, 250)
    ax.set_title("Feature space: energy (A) vs intra-window ripple (B)\n"
                 "(x = arc classes, o = normal classes)")
    # the one normal class that lands inside the arc cluster here is
    # pwm_switch_on -- it is separated by feature C (inter-window
    # stationarity), which this 2-D projection cannot show
    ax.annotate("pwm_switch_on overlaps arcs in this plane;\n"
                "rejected by feature C (stationarity)",
                xy=(150, 15), xytext=(60, 120), fontsize=8,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG_DIR, "feature_space.png"), dpi=130)
    plt.close(fig)

    # ---- 4. persistence sweep tradeoff ----
    rows = np.array(sweep_rows, dtype=float)
    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    ax.plot(rows[:, 0] * 8, rows[:, 1] * 100, "o-", label="missed arcs")
    ax.plot(rows[:, 0] * 8, rows[:, 2] * 100, "s-", label="nuisance trips")
    ax.axvline(det.N_SLOW * 8, color="k", ls=":", lw=1,
               label=f"chosen ({det.N_SLOW * 8} ms)")
    ax.set_xlabel("persistence requirement N_SLOW [ms]")
    ax.set_ylabel("rate [%]")
    ax.set_title("Sensitivity vs nuisance tradeoff (persistence sweep)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(IMG_DIR, "tradeoff_sweep.png"), dpi=130)
    plt.close(fig)
    print(f"plots written to {os.path.abspath(IMG_DIR)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    np.random.seed(args.seed)
    print(f"=== Monte-Carlo validation: {args.trials} trials/class ===\n")
    results = run_trials(args.trials)
    report = print_report(results)

    print("\n=== persistence sweep (50 trials/class/point) ===")
    np.random.seed(args.seed + 1)
    rows = sweep_persistence(50)

    if not args.no_plots:
        make_plots(rows)

    out = os.path.join(os.path.dirname(__file__), "..", "docs", "validation-results.md")
    with open(out, "w") as f:
        f.write("# Validation results\n\n(generated by `sim/validate.py`, "
                f"{args.trials} Monte-Carlo trials per event class)\n\n```\n"
                + report + "\n```\n\nPersistence sweep:\n\n```\n")
        for n_slow, miss, nuis in rows:
            f.write(f"N_SLOW={n_slow:3d} ({n_slow*8:3d} ms): "
                    f"missed {miss*100:5.1f} %   nuisance {nuis*100:5.1f} %\n")
        f.write("```\n")
    print(f"report written to {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
