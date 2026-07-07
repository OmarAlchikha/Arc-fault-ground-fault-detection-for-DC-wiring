# Hardware design

Everything here is generic, public-knowledge circuit design using catalog
parts. Bench setting: a 24–28 V DC supply feeding a resistive/electronic
load at 1–5 A through the detector.

> **Safety note.** This is a student/portfolio bench project. It is not a
> certified protection device and must never be the only thing standing
> between a power source and a hazard. Always test behind a current-limited
> bench supply and an ordinary fuse. Deliberate arc generation should only
> be done at low energy (current-limited supply, small gaps, fire-safe
> surface, eye protection — arcs are bright and spatter).

## Block diagram

```
                 +-----------------------------+
  DC+ ----------->  hall sensor #1 (ACS723)     >---------+------> to load +
                 +-----------------------------+          |
                        | Vout (2.5V ± 400mV/A)           |
                        v                             +---+----+
        +---------------+----------------+           | MOSFET |  switching stage
        |               |                |           | stage  |  (perfboard)
   LF path         HF path          (raw, unused)    +---+----+
   Sallen-Key      C-coupled BPF                          |
   LPF 1.5 kHz     8–45 kHz, G=20                         |
        |               |                                 |
        |          precision rectifier                    |
        |               |                                 |
        |          RC envelope (τ=1 ms)                   |
        |               |                                 |
        v               v                                 |
       A0              A1        Arduino Mega 2560        |
                                  A2 <— hall sensor #2 ---+--- on RETURN wire
                                  D7 —> gate driver of switching stage
  DC- ---------------------------------------------------------> to load -
```

## 1. Current sensing — hall sensor, not a shunt

**Choice: ACS723LLCTR-05AB-class hall sensor (±5 A, 400 mV/A, 80 kHz BW),
one on the supply conductor, one on the return conductor.**

Why not a shunt + diff amp (the "obvious" first answer):

- **Galvanic isolation.** The measurement side never touches the power
  side. On a wiring-protection device that is worth a lot: a miswired or
  faulted line cannot pull the MCU rail around.
- **Common-mode simplicity.** A high-side shunt at 28 V needs a proper
  current-sense amplifier with good CMRR; a low-side shunt breaks the
  single-ground assumption the return-side GF sensor relies on.
- **The 80 kHz analog bandwidth is enough** because the detection band was
  deliberately chosen *below* it (8–45 kHz, see below).

Cost of the choice: hall sensors have more low-frequency noise and offset
drift than a shunt. The algorithm tolerates both — the HF path is
AC-coupled, and the LF path only feeds *relative* step detection, never an
absolute current threshold.

## 2. Why an analog HF front end at all

The core resource problem: DC arc noise is broadband (tens of kHz to MHz),
but an ATmega2560 tops out around 77 kS/s at reduced resolution — nowhere
near Nyquist for the interesting band, with no cycles left for DSP.

So the analog front end performs the one operation the MCU can't: it
**demodulates "how much broadband noise is present right now" into a slow
envelope**. Band-pass → rectify → RC low-pass is exactly an AM envelope
detector. The MCU then samples a signal whose information rate is a few
hundred Hz, and all "DSP" happens at 4 kHz in integer math.

This mirrors how real commercial AFCI/AFD ASICs are structured (log-detector
+ comparator front ends), and it is the single most important architectural
decision in the project.

## 3. Detection band: 8–45 kHz

- **Lower edge (8 kHz):** must sit above everything loads legitimately do
  at high energy: PWM fundamentals (100s of Hz to a few kHz), motor
  commutation ripple, supply ripple, and the LF path's content. One decade
  of separation keeps the band-pass skirts effective with a modest
  2nd-order filter.
- **Upper edge (45 kHz):** must sit below the sensor's 80 kHz bandwidth
  with margin (the sensor's response is already rolling off there), and
  below the strongest DC-DC converter fundamentals (many sit at 50–500 kHz
  — keeping the band *under* them helps, though harmonically-related
  content can still land in-band; see limitations).
- Arc conduction noise is broadband and strong through this whole region,
  so we lose little sensitivity by not chasing the MHz content.

## 4. HF path circuit (op-amp stages)

Dual op-amp, MCP6022-class (rail-to-rail, 10 MHz GBW — at G≈20 and 45 kHz
you need GBW ≥ ~1 MHz *at the top of the band*, so a 1 MHz "jellybean"
MCP6002 is NOT enough; this is an easy trap).

1. **AC coupling:** 100 nF from sensor output into the filter — strips the
   2.5 V offset and the load current itself.
2. **Band-pass:** 2nd-order multiple-feedback (MFB) band-pass centred
   ~20 kHz, Q ≈ 0.7 equivalent to the simulated 8–45 kHz Butterworth,
   gain ≈ 20. MFB is preferred over Sallen-Key here because its stopband
   doesn't "come back up" at high frequency through the feed-forward cap.
3. **Precision rectifier:** classic two-diode/two-op-amp absolute-value
   stage (1N4148s). A bare diode would lose everything below ~0.6 V — most
   of the useful signal.
4. **Envelope:** 10 kΩ / 100 nF → τ = 1 ms, buffered into A1.
   - τ trades ripple vs. responsiveness. 1 ms keeps 8 ms windows meaningful
     (window = 8τ) while letting impulsive events (contact bounce, PWM
     edges at ≤ 2 kHz) visibly sag between impulses — the continuity
     feature *depends* on that sag surviving.

**Gain budget (headroom, deliberately conservative):** net conversion
≈ 15 V per A of in-band noise. Sized so a weak 20 mA-RMS arc lands at
~250 mV (≈ 50 counts, comfortably above the 12-count floor) while the
nastiest simulated PWM load stays under ~4 V — because **clipping flattens
envelope ripple and destroys the shape features**. Early simulation runs
with 40 V/A saturated on PWM loads and directly caused false trips; the
gain reduction was a simulation-driven fix, not a guess.

## 5. LF path

Sallen-Key 2nd-order low-pass at 1.5 kHz, unity gain, straight into A0.
Anti-aliasing for the 4 kHz sampling and nothing more. The current-step
feature only needs ~9 % relative changes, so 10-bit resolution at 400 mV/A
(≈ 12 mA/count) is ample at 1–5 A loads.

## 6. Switching stage (the perfboard build)

**Topology: low-side logic-level N-MOSFET (IRLZ44N or similar) in the
return leg, driven through a small gate resistor, with a pull-down.**

- **Why a MOSFET and not a relay:** a relay opening 5 A of DC *draws its
  own arc* across the opening contacts — using an arc to clear an arc, and
  it can weld. DC has no zero-crossings to help. A MOSFET opens in
  microseconds with no contacts. (AC breakers get their arc quenched at
  the next zero-crossing; that luxury does not exist here. This is why DC
  switching is fundamentally harder than AC and why aerospace/solar DC
  systems care so much about it.)
- **Why low-side:** the gate can then be driven directly from a 5 V MCU
  pin (via ~100 Ω) referenced to common ground. A high-side switch would
  need a gate driver or charge pump — real products do this to keep the
  return conductor solid, and it's a known compromise of this build.
- **IRLZ44N sizing:** logic-level threshold, R_DS(on) ≈ 22 mΩ at
  V_GS = 5 V → ~0.5 W at 5 A, a small clip-on sink or free air is fine.
- **Fail-safe polarity:** gate pull-down 10 kΩ. MCU dead / resetting /
  unpowered ⇒ gate low ⇒ **line open**. The firmware also boots with the
  line open and only closes it after a sensing-chain self-test passes.
- **TVS (e.g. SMBJ33A) across the MOSFET and a flyback path across the
  load terminals:** interrupting current in an inductive line dumps
  L·di/dt into whatever is across the switch. Without clamping, the act
  of tripping could avalanche the MOSFET — the protector must survive its
  own protective action.
- **Fuse upstream anyway.** The MOSFET protects against arcs; the fuse
  protects against the MOSFET failing short. Layered protection, never a
  single device.

Perfboard practice notes: keep the power loop (sensor → MOSFET → terminals)
short and wide (solder-filled traces / bus wire), star the analog ground at
the sensor, keep the HF filter stage away from the switching node, decouple
each op-amp at the pin.

## 7. Ground-fault sensing

Second hall sensor on the return conductor; the MCU compares the two window
means. Healthy circuit: i_line = i_return. Current leaking to chassis/earth
(insulation failure, chafed wire to structure) shows up as a sustained
imbalance. Threshold ≈ 150 mA for 100 ms.

- 150 mA is far above the mismatch of two ±1 % sensors at 5 A? No — 1 % of
  5 A is 50 mA per sensor, so worst-case static mismatch approaches
  100 mA. 150 mA is chosen *because of* that error budget; a production
  design would use a single differential (core-balance) sensor with both
  conductors through one core, which cancels the error by construction.
  That is the honest limitation of the two-sensor approach and the first
  thing to upgrade.

## 8. Bill of materials (indicative)

| Qty | Part | Role |
|----:|------|------|
| 1 | Arduino Mega 2560 | detection + trip logic |
| 2 | ACS723-05AB (or ACS712-05 with noise caveats) | line + return current |
| 1 | MCP6022 (dual, 10 MHz RRIO) | band-pass + rectifier |
| 1 | MCP6002 (dual, 1 MHz) | LF anti-alias buffer + envelope buffer |
| 1 | IRLZ44N | disconnect switch |
| 2 | 1N4148 | precision rectifier |
| 1 | SMBJ33A TVS + 1N5822 flyback | clamping |
| — | Rs/Cs per filter values above, perfboard, terminal blocks, fuse+holder | — |

## 9. Bring-up order (how it was meant to be built)

1. Switching stage alone on perfboard: drive gate from a bench square wave,
   verify clean switching of a resistive load, check MOSFET temperature.
2. LF path: verify 400 mV/A scaling and step response with a load switch.
3. HF path: inject a signal-generator burst (20 kHz, few mV) at the AC
   coupling cap; verify envelope amplitude and τ.
4. Firmware with `PIN_TRIP` disconnected: log telemetry over serial while
   exercising loads; confirm baseline settles and counter stays at 0.
5. Only then connect the gate, and generate a real (small!) test arc:
   graphite pencil lead in series with the load, gently separated — the
   classic bench arc generator, behind a current-limited supply.
