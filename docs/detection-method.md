# Detection method

## What a DC arc fault looks like (public literature, condensed)

A series arc (loose terminal, broken strand, chafed conductor barely
touching) inserts a small plasma gap into the circuit. Public-domain
observations that the detector exploits:

1. **Arc voltage.** A short DC arc in air drops roughly 12–20 V nearly
   independent of current. In a series fault this voltage subtracts from
   what the load sees, so **load current steps down** at arc ignition.
   In a parallel fault (line-to-return through an arc) current steps *up*.
2. **Broadband conduction noise.** The arc column is chaotic — its
   conductance fluctuates continuously, superimposing broadband noise
   (kHz→MHz, roughly 1/f-shaped) on the line current **for as long as the
   arc burns**. This is the primary detectable signature.
3. **Non-stationarity.** The noise *amplitude itself* wanders on ms–100 ms
   scales (re-ignition, arc-root movement, electrode heating). An arc's
   noise level measured in adjacent 8 ms windows differs a lot;
   man-made interference is far more stationary.
4. **DC has no zero-crossings.** Unlike AC, nothing periodically
   extinguishes the arc — a stable DC arc can burn indefinitely, which is
   both why it is so dangerous and why "sustained" is a usable criterion.

## Sensed quantities

Two ADC channels at 4 kHz each (plus a return-current channel for ground
fault). The analog front end (docs/hardware.md) reduces the MHz problem to
a kHz problem:

- `i_lf` — low-pass-filtered load current (< 1.5 kHz).
- `hf_env` — envelope of in-band (8–45 kHz) noise: "how much broadband
  noise is on the line *right now*", as a slow voltage.

## Features (per 8 ms window, 32 samples/channel, integer math)

| Feature | Computation | Meaning |
|---|---|---|
| A: energy | `hf_mean > baseline + max(4·dev, 12)` | more in-band noise than this line's own quiet level |
| B: continuity | `hf_mad·2 < (hf_mean − baseline)` | envelope *held up* through the window, not spiky |
| C: chaos | `asd_ema·16 > (hf_mean − baseline)` | envelope level *wanders between* windows |
| corroboration | window-mean current step ≥ ~9 % (≥ 8 counts) | arc ignition kicked the load current |

`baseline`/`dev` are exponential moving averages (τ ≈ 0.5 s) **updated only
during quiet windows** — an arc must never teach the detector that arcing
is normal. `asd_ema` is an EMA (α = 1/4) of |Δ hf_mean| between consecutive
windows.

A window is *arc-suspect* iff **A ∧ B ∧ C**. A persistence counter does
+1/−1 (capped at 20, floored at 0):

- **Slow path:** counter ≥ 18 (≈ 144 ms of majority-suspect windows) → trip.
- **Fast path:** counter ≥ 9 (≈ 72 ms) *and* a significant current step in
  the last 200 ms → trip. Series/parallel arcs almost always announce
  themselves with a step, so real faults usually take this path.

Trip opens the MOSFET, latches, and requires a button reset.

## Why each nuisance source is rejected

| Event | What it produces | Rejected by |
|---|---|---|
| Steady load, supply ripple | nothing in 8–45 kHz | A |
| Load switch w/ contact bounce | genuine micro-arcs! but impulsive and over in 1–4 ms | B (envelope sags between bounces) + persistence |
| Inrush (capacitor/lamp) | one switching edge of HF, huge but smooth di/dt | A after first window; di/dt alone is deliberately *not* a trip criterion |
| PWM load, slow (≤ ~2 kHz) | strong HF at every edge, envelope ripples at PWM rate | B |
| PWM/converter, fast (edges merge) | steady elevated envelope | C (stationary) |
| Brushed motor | weak commutation micro-arcs, amplitude-modulated | mostly below A; residual risk documented below |

### The design history that matters (and would matter in a review)

The first cut used **one** shape feature — intra-window MAD — assuming "arcs
are chaotic ⇒ high variance." Monte-Carlo simulation falsified this twice:

1. At envelope timescales arcs are the *smooth* ones (continuous noise
   holds the envelope up: MAD/elevation ≈ 0.03–0.35), while impulsive
   interference sawtooths (0.7–5). So the feature was inverted into the
   **continuity** test.
2. Continuity alone then failed the *PWM-switches-on-mid-run* case: a fast
   PWM's merged envelope is also smooth and steps up against a quiet
   baseline, exactly like an arc. The separating physics is
   **stationarity**: the PWM envelope is flat window-to-window
   (|Δ|/elevation < 0.09 for ~90 % of windows) while arc noise wanders
   (0.10–0.30). That became the **chaos** test.
3. An early front-end gain (40 V/A) let strong PWM loads clip the envelope
   ADC; clipping flattens ripple and *manufactures* arc-like smoothness.
   Gain was cut to 15 V/A specifically to keep the worst interferer
   linear. **Headroom is a detection feature.**

Arc = the only source that is simultaneously **elevated (A), continuous
(B), and non-stationary (C)**.

## Validated performance (sim/validate.py, 200 trials/class)

See `docs/validation-results.md` for the generated numbers and
`docs/img/*.png` for plots. Summary at the chosen operating point
(N_SLOW = 18, N_FAST = 9):

- series/parallel arcs: ≥ 98 % detected, median trip ≈ 75 ms, p95 < 170 ms
- arc igniting under a running PWM load: 100 % detected, median ≈ 150 ms
- all nuisance classes: 0 % false trips **except** PWM-switch-on ≈ 2–3 %
- persistence sweep shows the tradeoff cliff: N ≤ 10 → nuisance > 16 %;
  N = 24 → ~19 % of arcs missed. 18 sits between them, biased toward
  catching arcs (a missed arc is a fire; a nuisance trip is an annoyance —
  on a bench. In an aircraft the calculus is different and documented
  below.)

## Limitations and tradeoffs (read this section first in any review)

1. **PWM load stepping on ≈ 3 % nuisance rate.** The residual failure mode:
   the switch-on transient itself pumps `asd_ema` for a few windows.
   Mitigations not implemented: hold off the fast path for ~100 ms after
   any current step *upward* (arcs from new loads are rare in the first
   100 ms); or add envelope autocorrelation to detect periodicity
   explicitly (costs RAM/cycles).
2. **Continuous in-band interference.** A converter whose fundamental or
   strong harmonics land in 8–45 kHz *and* whose amplitude jitters (e.g.
   hysteretic/burst-mode controllers) can satisfy A ∧ B ∧ C. A production
   device adds load-current-correlation and periodicity analysis; final
   answer is usually "and we qualified against a library of real loads,"
   which is exactly what UL 1699-style arc generators + load banks are for.
3. **Brushed motors are the classic AFCI nightmare.** Healthy commutation
   noise passed here because it is weak and amplitude-modulated, but a
   *worn* motor is physically arcing — no signature-based detector can
   fully separate "motor that arcs by design" from "wiring that arcs by
   fault." Real systems whitelist/zone such loads.
4. **Masking.** A strong PWM load raises the adaptive baseline; a weak arc
   behind it may fall below feature A. Visible in the sim: `arc_during_pwm`
   trips ~2× slower. Fundamental to any energy-based method.
5. **Detection floor.** Very-low-current arcs (< ~0.5 A load) produce noise
   near the 12-count floor; the floor exists to tolerate sensor/EMI noise,
   so sensitivity and immunity trade directly through it.
6. **Simulation is not reality.** The waveform models are physically
   motivated but synthetic; parameters (arc noise 1.5–6 % of load RMS,
   bounce timing, etc.) are from public literature ranges. The algorithm's
   *structure* is validated; its *thresholds* must be re-tuned against real
   bench arcs (pencil-lead / opening-contact generator) before any claim
   about real-world rates. The persistence sweep exists precisely to show
   where the knobs are.
7. **Single-point protection.** The detector itself (sensor, op-amps, MCU)
   is unmonitored beyond the boot self-test; a real device needs continuous
   self-supervision (test-tone injection into the HF path is the standard
   trick) and a redundant trip path.
8. **Trip time is statistical, not guaranteed.** p95 ≈ 165 ms in sim, but
   the persistence counter means a marginal, sputtering arc can dance below
   threshold indefinitely. Standards handle this with energy-integral
   requirements (i²t-style limits); implementing an accumulated-energy trip
   in parallel would bound worst-case let-through.

## Nuisance-trip philosophy

On the bench, bias toward sensitivity. In an aviation-adjacent mindset the
tradeoff inverts per circuit criticality: nuisance-tripping a cabin light
is acceptable; nuisance-tripping something flight-critical is itself a
hazard, which is why arc-fault protection in aerospace is deployed
selectively and with per-load qualification. The right engineering answer
is not one threshold — it is: **the tradeoff is a tunable, documented curve
(docs/img/tradeoff_sweep.png), and the operating point is a system-level
decision, not a firmware default.**
