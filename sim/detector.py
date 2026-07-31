"""
Reference implementation of the arc-fault detection algorithm.

This is a line-for-line mirror of the logic in
firmware/arc_fault_detector/arc_fault_detector.ino, written with the same
integer arithmetic the ATmega2560 will use, so that what we validate here
is what actually ships.  Keep the two in sync.

Algorithm summary (full rationale in docs/detection-method.md):

  Per 8 ms window (32 samples/channel @ 4 kHz):
    hf_mean : mean of HF-envelope channel        -> "how much broadband noise"
    hf_mad  : mean absolute deviation of same    -> intra-window ripple
    asd_ema : EMA of |hf_mean - previous hf_mean| -> inter-window instability
    i_mean  : mean of LF current channel         -> load current level

  Adaptive baseline (EMA, tau ~ 0.5 s) tracks hf_mean; frozen while any
  window is suspicious so the arc can't teach the detector that arcing
  is normal.

  A window is an ARC-SUSPECT window iff all three hold
  (hf_ac = hf_mean - baseline, the above-baseline elevation):
    (A) hf_mean > baseline + max(K_REL * baseline_dev, ABS_FLOOR)  [energy]
    (B) hf_mad * CONT_NUM < hf_ac                             [continuity]
    (C) asd_ema * CHAOS_DEN > hf_ac                                [chaos]

  Features (B) and (C) are a matched pair, and the split matters -- the
  first single-feature design was wrong in both directions and simulation
  caught it (full story in docs/detection-method.md):

    (B) continuity, intra-window: arc conduction noise is CONTINUOUS in
        time, so after the 1 ms envelope RC it holds the envelope up
        smoothly within a window (MAD/elevation 0.03-0.35).  Impulsive
        events -- switch-edge bursts, contact-bounce micro-arcs, slow PWM
        edges -- let the envelope sawtooth back toward zero between
        impulses (ratio 0.7-5).  Threshold at 0.5.

    (C) chaos, inter-window: arc noise amplitude is NON-STATIONARY
        (re-ignition, wandering arc root), so window-to-window hf_mean
        jumps around (|delta|/elevation typically 0.10-0.30).  A fast PWM
        or converter puts steady periodic energy in band: elevated but
        STATIONARY envelope (ratio < 0.09 for ~90 % of windows).
        Threshold at 1/16 = 0.0625.

  Persistence counter: +1 on suspect window, -1 (floor 0) otherwise.
    trip if counter >= N_SLOW                    (18 windows, 144 ms)
    trip if counter >= N_FAST and a coincident   ( 9 windows,  72 ms)
      current-step signature was seen (series arc drop / parallel arc jump)

  Why this rejects the nuisance cases:
    - contact bounce: real micro-arcs, but impulsive -> (B) fails, and over
      in 1-4 ms so the counter never builds anyway
    - inrush: one switching edge of HF, then spectrally quiet; big di/dt is
      deliberately NOT a trip criterion on its own
    - slow PWM (edges resolved by the envelope): ripple at PWM rate -> (B)
    - fast PWM / converter (edges merge into a steady band): stationary
      envelope -> (C) fails
    - brushed motor: weak, amplitude-modulated noise; mostly below (A)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ---- tuning constants (mirror #defines in the firmware) --------------------
WIN = 32                 # samples per window per channel (8 ms @ 4 kHz)
K_REL_NUM = 4            # energy threshold: baseline + 4*dev
ABS_FLOOR = 12           # ...but never less than +12 counts (~60 mV)
CONT_NUM = 2             # continuity test: mad*2 < (mean-baseline), i.e. ratio < 0.5
CHAOS_DEN = 16           # chaos test: asd_ema*16 > (mean-baseline), i.e. ratio > 1/16
ASD_ALPHA_SHIFT = 2      # asd EMA: alpha = 1/4 per window
BASE_ALPHA_SHIFT = 6     # baseline EMA: alpha = 1/64 per window  (tau ~ 0.5 s)
DEV_ALPHA_SHIFT = 5      # deviation EMA: alpha = 1/32
N_FAST = 9               # windows (72 ms) if corroborated by current step
N_SLOW = 18              # windows (144 ms) on HF signature alone
STEP_FRAC_NUM = 6        # current step counts if |di| > i_prev*6/64 (~9%)
STEP_MIN = 8             # ...and at least 8 ADC counts (~50 mA)
STEP_MEMORY = 25         # windows (200 ms) a current step stays "recent"
COUNTER_MAX = 20         # cap so a long nuisance can't bank credit


@dataclass
class DetectorState:
    baseline: int = 0        # Q8 fixed point (counts << 8)
    base_dev: int = 0        # Q8
    asd_ema: int = 0         # Q4 fixed point (counts << 4)
    hf_prev: int = -1
    counter: int = 0
    step_age: int = 999      # windows since last significant current step
    i_prev: int = -1
    tripped: bool = False
    trip_window: int = -1
    initialized: bool = False


def _window_features(hf: np.ndarray, ilf: np.ndarray) -> tuple[int, int, int]:
    """Integer features exactly as the AVR computes them."""
    hf = hf.astype(np.int32)
    hf_mean = int(np.sum(hf)) // WIN
    hf_mad = int(np.sum(np.abs(hf - hf_mean))) // WIN
    i_mean = int(np.sum(ilf.astype(np.int32))) // WIN
    return hf_mean, hf_mad, i_mean


def process_trace(hf_env: np.ndarray, i_lf: np.ndarray,
                  collect: bool = False):
    """Run the detector over a full recording.  Returns (state, log) where
    log is a per-window dict-of-arrays when collect=True."""
    st = DetectorState()
    n_win = len(hf_env) // WIN
    log = {k: [] for k in ("hf_mean", "hf_mad", "i_mean", "baseline",
                           "suspect", "counter")} if collect else None

    for w in range(n_win):
        s = slice(w * WIN, (w + 1) * WIN)
        hf_mean, hf_mad, i_mean = _window_features(hf_env[s], i_lf[s])

        # ---- startup: seed baseline from the first few windows ----
        if not st.initialized:
            if st.baseline == 0:
                st.baseline = hf_mean << 8
                st.base_dev = 4 << 8
            else:
                st.baseline += ((hf_mean << 8) - st.baseline) >> 2
            if w >= 8:
                st.initialized = True
            st.i_prev = i_mean
            st.hf_prev = hf_mean
            if collect:
                for k, v in (("hf_mean", hf_mean), ("hf_mad", hf_mad),
                             ("i_mean", i_mean), ("baseline", st.baseline >> 8),
                             ("suspect", 0), ("counter", 0)):
                    log[k].append(v)
            continue

        base = st.baseline >> 8
        dev = st.base_dev >> 8

        # ---- feature A: sustained above-baseline HF energy ----
        thresh = base + max((K_REL_NUM * dev), ABS_FLOOR)
        energy = hf_mean > thresh

        # ---- feature B: continuity (sustained, not impulsive, elevation) ----
        hf_ac = hf_mean - base
        continuity = (hf_mad * CONT_NUM) < max(hf_ac, 1)

        # ---- feature C: chaos (inter-window amplitude instability) ----
        asd = abs(hf_mean - st.hf_prev)
        st.hf_prev = hf_mean
        st.asd_ema += ((asd << 4) - st.asd_ema) >> ASD_ALPHA_SHIFT
        chaos = ((st.asd_ema >> 4) * CHAOS_DEN) > max(hf_ac, 1)

        suspect = energy and continuity and chaos

        # ---- current-step corroboration (series drop OR parallel jump) ----
        if st.i_prev >= 0:
            di = abs(i_mean - st.i_prev)
            load = abs(st.i_prev - 512)  # counts away from 2.5 V zero-current
            if di > max((load * STEP_FRAC_NUM) >> 6, STEP_MIN):
                st.step_age = 0
        st.i_prev = i_mean

        # ---- baseline adaptation: only learn from quiet windows ----
        if not energy:
            st.baseline += ((hf_mean << 8) - st.baseline) >> BASE_ALPHA_SHIFT
            err = abs(hf_mean - (st.baseline >> 8))
            st.base_dev += ((err << 8) - st.base_dev) >> DEV_ALPHA_SHIFT
            if st.base_dev < (1 << 8):
                st.base_dev = 1 << 8

        # ---- persistence and trip decision ----
        if suspect:
            st.counter = min(st.counter + 1, COUNTER_MAX)
        else:
            st.counter = max(st.counter - 1, 0)

        if not st.tripped:
            if st.counter >= N_SLOW or \
               (st.counter >= N_FAST and st.step_age <= STEP_MEMORY):
                st.tripped = True
                st.trip_window = w

        if st.step_age < 999:
            st.step_age += 1

        if collect:
            for k, v in (("hf_mean", hf_mean), ("hf_mad", hf_mad),
                         ("i_mean", i_mean), ("baseline", base),
                         ("suspect", int(suspect)), ("counter", st.counter)):
                log[k].append(v)

    if collect:
        log = {k: np.array(v) for k, v in log.items()}
    return st, log


def trip_time_s(state: DetectorState, arc_onset_s: float | None) -> float | None:
    """Seconds from arc onset to trip, or None if no trip."""
    if not state.tripped:
        return None
    t_trip = (state.trip_window + 1) * WIN / 4000.0
    if arc_onset_s is None:
        return t_trip
    return t_trip - arc_onset_s
