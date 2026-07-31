"""
Representative waveform generator for DC series/parallel arc faults and
normal (non-fault) load events, plus a model of the analog front end.

Everything here is public-knowledge engineering: arc noise characteristics
(broadband, chaotic, sustained), inrush profiles, and contact-bounce
behaviour are textbook / published-literature material.

Signal chain being modelled
---------------------------
                       +--> LF path: LPF (<2 kHz) -----------------> ADC ch0 "i_lf"
  I(t) -> shunt/hall --+
                       +--> HF path: BPF 8-45 kHz -> |abs| -> RC ---> ADC ch1 "hf_env"

The MCU (Arduino Mega) samples each channel at FS_ADC = 4 kHz.  The
high-frequency content of the arc is *not* sampled directly -- the analog
band-pass + envelope detector converts "how much broadband noise is on the
line right now" into a slow voltage the 10-bit AVR ADC can follow.  That
hardware/firmware split is the core architectural decision of the project
(see README).

Simulation is run at FS_SIM = 200 kHz so the 8-45 kHz band is well
represented.  Real arc noise extends into the MHz range; band-limiting the
model to <100 kHz is a deliberate simplification and is conservative for
the detector (less arc energy available, not more).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import signal

# ----------------------------------------------------------------------------
# Constants shared with the firmware (keep in sync with arc_fault_detector.ino)
# ----------------------------------------------------------------------------
FS_SIM = 200_000          # simulation sample rate [Hz]
FS_ADC = 4_000            # per-channel MCU sample rate [Hz]
DECIM = FS_SIM // FS_ADC  # 50

V_BUS = 28.0              # nominal DC bus voltage [V] (28 V is a common public DC standard)
ADC_FS_V = 5.0            # ADC full scale [V]
ADC_BITS = 10

# Current sensor model: hall-effect, bidirectional, 2.5 V offset.
# 400 mV/A (ACS723-05 class) => +/-5 A usable range on a 5 V ADC.
SENS_V_PER_A = 0.400
SENS_OFFSET_V = 2.500

# HF envelope path net gain.  Sized for HEADROOM, not sensitivity: the
# strongest anticipated interferer (hard-switched PWM edges) must stay in
# the ADC's linear range, because clipping flattens the envelope ripple and
# destroys the continuity discriminator (feature B).  15 V/A puts a weak
# 20 mA-RMS arc at ~250 mV (well above the ~30 mV floor) and a heavy PWM
# load at ~3-4 V (below the 5 V rail).  See docs/hardware.md.
HF_GAIN_V_PER_A = 15.0    # net: sensor HF response * BPF gain * rectifier
HF_ENV_TAU = 1.0e-3       # envelope RC time constant [s]

BPF_LO, BPF_HI = 8_000.0, 45_000.0


@dataclass
class Trace:
    """One simulated event, as the MCU sees it."""
    name: str
    label: str                     # "arc" or "normal"
    i_true: np.ndarray             # ground-truth line current at FS_SIM [A]
    i_lf: np.ndarray               # ADC ch0 counts at FS_ADC (LF current)
    hf_env: np.ndarray             # ADC ch1 counts at FS_ADC (HF envelope)
    arc_onset_s: float | None = None   # when the arc actually starts (for trip-time stats)
    meta: dict = field(default_factory=dict)

    @property
    def t_adc(self) -> np.ndarray:
        return np.arange(len(self.i_lf)) / FS_ADC


# ----------------------------------------------------------------------------
# Analog front end model
# ----------------------------------------------------------------------------
_sos_bpf = signal.butter(2, [BPF_LO, BPF_HI], btype="bandpass", fs=FS_SIM, output="sos")
_sos_lf = signal.butter(2, 1_500.0, btype="lowpass", fs=FS_SIM, output="sos")


def _quantize(v: np.ndarray) -> np.ndarray:
    """5 V / 10-bit ADC with clipping, plus ~1 LSB of conversion noise."""
    counts = v / ADC_FS_V * (2**ADC_BITS - 1)
    counts = counts + np.random.normal(0.0, 0.6, size=counts.shape)
    return np.clip(np.round(counts), 0, 2**ADC_BITS - 1)


def front_end(i_line: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Run true line current through the modelled analog chain, return
    (i_lf_counts, hf_env_counts) at the ADC rate."""
    # --- LF current path ---
    v_lf = SENS_OFFSET_V + SENS_V_PER_A * signal.sosfilt(_sos_lf, i_line)

    # --- HF noise path: band-pass -> full-wave rectify -> RC envelope ---
    hf = signal.sosfilt(_sos_bpf, i_line)
    rect = np.abs(hf) * HF_GAIN_V_PER_A
    # single-pole RC envelope
    alpha = 1.0 / (FS_SIM * HF_ENV_TAU)
    env = signal.lfilter([alpha], [1.0, -(1.0 - alpha)], rect)
    # small DC pedestal from rectifier offset / ambient EMI floor
    env += 0.030 + np.random.normal(0, 0.002, size=env.shape)

    # decimate to MCU rate (ADC just samples the already-slow signals)
    return _quantize(v_lf[::DECIM]), _quantize(np.clip(env, 0, ADC_FS_V)[::DECIM])


# ----------------------------------------------------------------------------
# Noise building blocks
# ----------------------------------------------------------------------------
def _band_noise(n: int, rms: float) -> np.ndarray:
    """Broadband noise shaped ~1/f across the detection band -- the classic
    published spectral shape of arc conduction noise."""
    w = np.random.normal(0, 1, n)
    # pinkish tilt: one-pole shelf
    b, a = [1.0], [1.0, -0.35]
    x = signal.lfilter(b, a, w)
    x = signal.sosfilt(_sos_bpf, x)  # only in-band content matters downstream
    r = np.sqrt(np.mean(x**2))
    return x / (r + 1e-12) * rms


def _burst(n_total: int, start: int, dur: int, rms: float) -> np.ndarray:
    """Short decaying broadband burst (switch edge, bounce micro-arc)."""
    out = np.zeros(n_total)
    dur = min(dur, n_total - start)
    if dur <= 0:
        return out
    env = np.exp(-np.arange(dur) / (dur / 3.0))
    out[start:start + dur] = _band_noise(dur, rms) * env
    return out


# ----------------------------------------------------------------------------
# Event generators.  Each returns a Trace.
# Parameters are randomized per call so validate.py can Monte-Carlo them.
# ----------------------------------------------------------------------------
def _sensor_floor(n: int) -> np.ndarray:
    """Ambient conducted-EMI / sensor noise floor present in every trace."""
    return _band_noise(n, rms=np.random.uniform(0.002, 0.006))


def steady_load(dur_s: float = 0.5) -> Trace:
    """Resistive load running quietly -- baseline case."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 4.5)
    i = np.full(n, i0) + _sensor_floor(n)
    # slow supply ripple
    t = np.arange(n) / FS_SIM
    i += 0.02 * i0 * np.sin(2 * np.pi * 120 * t)
    lf, hf = front_end(i)
    return Trace("steady_load", "normal", i, lf, hf, meta={"i0": i0})


def load_step(dur_s: float = 0.5) -> Trace:
    """Mechanical switch closes onto a resistive load, WITH contact bounce.
    Bounce produces genuine micro-arcs -- brief broadband bursts that a naive
    HF-energy detector will trip on.  This is the canonical nuisance case."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 4.5)
    t_close = int(np.random.uniform(0.15, 0.25) * FS_SIM)
    i = np.zeros(n)
    i[t_close:] = i0

    # contact bounce: 3-8 make/break cycles over 0.5-4 ms
    n_bounce = np.random.randint(3, 9)
    tb = t_close
    noise = np.zeros(n)
    for _ in range(n_bounce):
        gap = int(np.random.uniform(0.05e-3, 0.5e-3) * FS_SIM)
        make = int(np.random.uniform(0.05e-3, 0.4e-3) * FS_SIM)
        if tb + gap + make >= n:
            break
        i[tb:tb + gap] = 0.0                      # contact open
        # micro-arc burst at each break/make edge
        noise += _burst(n, tb, int(0.3e-3 * FS_SIM), rms=np.random.uniform(0.02, 0.08))
        tb += gap + make
    i += noise + _sensor_floor(n)
    lf, hf = front_end(i)
    return Trace("load_step_bounce", "normal", i, lf, hf, meta={"i0": i0, "bounces": n_bounce})


def inrush(dur_s: float = 0.5) -> Trace:
    """Capacitive/lamp inrush: 5-10x current spike decaying over 5-40 ms.
    Huge di/dt but spectrally quiet after the first edge."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 4.0)
    peak = i0 * np.random.uniform(4.0, 9.0)
    tau = np.random.uniform(3e-3, 25e-3)
    t_on = int(np.random.uniform(0.15, 0.25) * FS_SIM)
    t = np.arange(n - t_on) / FS_SIM
    i = np.zeros(n)
    i[t_on:] = i0 + (peak - i0) * np.exp(-t / tau)
    # single switching edge burst (no bounce: assume solid-state or good relay)
    i += _burst(n, t_on, int(0.2e-3 * FS_SIM), rms=np.random.uniform(0.02, 0.06))
    i += _sensor_floor(n)
    lf, hf = front_end(i)
    return Trace("inrush", "normal", i, lf, hf, meta={"peak": peak, "tau": tau})


def pwm_load(dur_s: float = 0.5) -> Trace:
    """Hard-switched PWM load (motor drive / LED dimmer / DC-DC input).
    Every edge injects HF energy, but PERIODICALLY -- the envelope is
    elevated yet steady.  The chaos feature is what rejects this."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 4.0)
    f_pwm = np.random.uniform(400, 2000)
    duty = np.random.uniform(0.3, 0.8)
    t = np.arange(n) / FS_SIM
    sq = (np.mod(t * f_pwm, 1.0) < duty).astype(float)
    i = i0 * (0.4 + 0.6 * sq)  # inductance keeps current from reaching zero
    # HF burst at each edge
    edges = np.where(np.abs(np.diff(sq)) > 0)[0]
    noise = np.zeros(n)
    burst_rms = np.random.uniform(0.015, 0.05)
    for e in edges:
        noise += _burst(n, e, int(0.08e-3 * FS_SIM), rms=burst_rms)
    i += noise + _sensor_floor(n)
    lf, hf = front_end(i)
    return Trace("pwm_load", "normal", i, lf, hf, meta={"f_pwm": f_pwm, "duty": duty})


def pwm_switch_on(dur_s: float = 0.6) -> Trace:
    """PWM load that turns ON mid-recording, against a quiet baseline.
    Harder than pwm_load: the detector cannot rely on its adaptive baseline
    having already absorbed the PWM noise -- the envelope steps up and STAYS
    up.  Only the ripple-shape (continuity) test separates it from an arc,
    which is why the HF path gain must keep this event out of clipping."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 4.0)
    f_pwm = np.random.uniform(400, 2000)
    duty = np.random.uniform(0.3, 0.8)
    t_on = int(np.random.uniform(0.15, 0.25) * FS_SIM)
    t = np.arange(n) / FS_SIM
    sq = (np.mod(t * f_pwm, 1.0) < duty).astype(float)
    sq[:t_on] = 0.0
    i = i0 * (0.4 + 0.6 * sq)
    i[:t_on] = 0.0
    edges = np.where(np.abs(np.diff(sq)) > 0)[0]
    noise = np.zeros(n)
    burst_rms = np.random.uniform(0.015, 0.05)
    for e in edges:
        noise += _burst(n, e, int(0.08e-3 * FS_SIM), rms=burst_rms)
    i += noise + _sensor_floor(n)
    lf, hf = front_end(i)
    return Trace("pwm_switch_on", "normal", i, lf, hf,
                 meta={"f_pwm": f_pwm, "duty": duty})


def brushed_motor(dur_s: float = 0.5) -> Trace:
    """Brushed DC motor: commutation noise IS micro-arcing, modulated at the
    commutation rate.  The classic hard case for any AFCI -- included so the
    validation reports an honest nuisance-trip number for it."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 3.0)
    f_comm = np.random.uniform(200, 800)  # commutation events per second
    t = np.arange(n) / FS_SIM
    # commutation ripple on the LF current
    i = i0 * (1 + 0.08 * signal.sawtooth(2 * np.pi * f_comm * t))
    # brush noise: broadband but amplitude-modulated periodically, and much
    # weaker than a fault arc (healthy brushes barely arc)
    am = 0.5 * (1 + np.sin(2 * np.pi * f_comm * t))
    i += _band_noise(n, rms=np.random.uniform(0.004, 0.015)) * am
    i += _sensor_floor(n)
    lf, hf = front_end(i)
    return Trace("brushed_motor", "normal", i, lf, hf, meta={"f_comm": f_comm})


def series_arc(dur_s: float = 0.5) -> Trace:
    """Series arc fault: loose terminal / broken strand in series with the load.
    Signature: (1) current DROPS because 12-20 V of arc voltage appears in
    series; (2) sustained, chaotic broadband noise; (3) random re-ignition
    spikes and occasional brief extinctions."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 4.5)
    r_load = V_BUS / i0
    t_arc = int(np.random.uniform(0.15, 0.25) * FS_SIM)

    # arc voltage: slow random walk between ~12 and ~20 V (published range
    # for short-gap DC arcs in air at these currents)
    v_arc = np.zeros(n)
    v = np.random.uniform(13, 18)
    steps = np.random.normal(0, 0.15, n)
    for k in range(t_arc, n):
        v = np.clip(v + steps[k], 11.0, 21.0)
        v_arc[k] = v

    i = np.full(n, i0)
    i[t_arc:] = (V_BUS - v_arc[t_arc:]) / r_load

    # sustained chaotic broadband noise, amplitude itself randomly modulated
    arc_rms = np.random.uniform(0.015, 0.06) * i0
    mod = np.clip(1 + 0.7 * signal.sosfilt(
        signal.butter(1, 300, fs=FS_SIM, output="sos"),
        np.random.normal(0, 12, n)), 0.1, 3.0)
    noise = _band_noise(n, arc_rms) * mod
    noise[:t_arc] = 0.0

    # random extinction/re-ignition events: brief current dips + spikes
    n_events = np.random.randint(3, 12)
    for _ in range(n_events):
        te = np.random.randint(t_arc, n - 200)
        dur = np.random.randint(20, 200)  # 0.1-1 ms
        i[te:te + dur] *= np.random.uniform(0.0, 0.5)
        noise += _burst(n, te, 100, rms=arc_rms * 2)

    i += noise + _sensor_floor(n)
    lf, hf = front_end(i)
    return Trace("series_arc", "arc", i, lf, hf,
                 arc_onset_s=t_arc / FS_SIM, meta={"i0": i0, "arc_rms": arc_rms})


def parallel_arc(dur_s: float = 0.5) -> Trace:
    """Parallel arc fault: chafed insulation shorting line-to-return through
    an arc.  Current INCREASES (bus / arc impedance) with the same chaotic
    noise.  Often higher energy than series arcs but may still sit below the
    breaker's magnetic trip point -- which is why AFDs exist."""
    n = int(dur_s * FS_SIM)
    i0 = np.random.uniform(1.0, 3.0)
    t_arc = int(np.random.uniform(0.15, 0.25) * FS_SIM)
    # fault path: wiring resistance + arc; draws an extra 3-10 A
    i_fault = np.random.uniform(3.0, 10.0)
    i = np.full(n, i0)
    i[t_arc:] += i_fault

    arc_rms = np.random.uniform(0.05, 0.20) * i_fault / 3
    mod = np.clip(1 + 0.7 * signal.sosfilt(
        signal.butter(1, 300, fs=FS_SIM, output="sos"),
        np.random.normal(0, 12, n)), 0.1, 3.0)
    noise = _band_noise(n, arc_rms) * mod
    noise[:t_arc] = 0.0
    n_events = np.random.randint(4, 15)
    for _ in range(n_events):
        te = np.random.randint(t_arc, n - 200)
        i[te:te + int(np.random.randint(20, 150))] *= np.random.uniform(0.4, 0.9)
        noise += _burst(n, te, 100, rms=arc_rms * 2)

    i += noise + _sensor_floor(n)
    lf, hf = front_end(i)
    return Trace("parallel_arc", "arc", i, lf, hf,
                 arc_onset_s=t_arc / FS_SIM, meta={"i0": i0, "i_fault": i_fault})


def arc_during_load(dur_s: float = 0.7) -> Trace:
    """Series arc that begins while a PWM load is also running -- the
    superposition case.  Documented hard case: periodic noise partially
    masks the chaos discriminator."""
    base = pwm_load(dur_s)
    n = len(base.i_true)
    t_arc = int(0.45 * dur_s * FS_SIM)
    arc_rms = np.random.uniform(0.03, 0.08) * np.mean(base.i_true)
    mod = np.clip(1 + 0.7 * signal.sosfilt(
        signal.butter(1, 300, fs=FS_SIM, output="sos"),
        np.random.normal(0, 12, n)), 0.1, 3.0)
    noise = _band_noise(n, arc_rms) * mod
    noise[:t_arc] = 0.0
    i = base.i_true + noise
    # arc series voltage knocks ~0.5 V-worth of current off the mean
    i[t_arc:] -= 0.15 * np.mean(base.i_true)
    lf, hf = front_end(i)
    return Trace("arc_during_pwm", "arc", i, lf, hf,
                 arc_onset_s=t_arc / FS_SIM, meta=base.meta)


NORMAL_GENERATORS = [steady_load, load_step, inrush, pwm_load,
                     pwm_switch_on, brushed_motor]
ARC_GENERATORS = [series_arc, parallel_arc, arc_during_load]
ALL_GENERATORS = NORMAL_GENERATORS + ARC_GENERATORS
