/*
 * detector_core.h — hardware-independent arc-fault detection core.
 *
 * Pure C99, no Arduino/AVR dependencies, so the exact code that runs on the
 * ATmega2560 can also be compiled on a PC and checked bit-for-bit against
 * the Python reference (sim/detector.py) using exported test vectors
 * (test/host_test.cpp).  If you change a constant here, change it in
 * sim/detector.py and re-run both validate.py and the host test.
 */
#ifndef DETECTOR_CORE_H
#define DETECTOR_CORE_H

#include <stdint.h>

/* ---- tuning constants (validated by sim/validate.py) ------------------- */
#define AF_WIN              32  /* samples per window per channel (8 ms @ 4 kHz) */
#define AF_K_REL_NUM         4  /* energy: baseline + 4*dev ...                  */
#define AF_ABS_FLOOR        12  /* ... but at least +12 counts (~60 mV)          */
#define AF_CONT_NUM          2  /* continuity: mad*2 < elevation (ratio < 0.5)   */
#define AF_CHAOS_DEN        16  /* chaos: asd_ema*16 > elevation (ratio > 1/16)  */
#define AF_BASE_ALPHA_SHIFT  6  /* baseline EMA alpha = 1/64 (tau ~ 0.5 s)       */
#define AF_DEV_ALPHA_SHIFT   5  /* deviation EMA alpha = 1/32                    */
#define AF_ASD_ALPHA_SHIFT   2  /* inter-window ASD EMA alpha = 1/4              */
#define AF_N_FAST            9  /* windows (72 ms) with current-step corroboration */
#define AF_N_SLOW           18  /* windows (144 ms) on HF signature alone        */
#define AF_STEP_FRAC_NUM     6  /* current step: |di| > load*6/64 (~9 %) ...     */
#define AF_STEP_MIN          8  /* ... and at least 8 counts (~50 mA)            */
#define AF_STEP_MEMORY      25  /* windows (200 ms) a step stays "recent"        */
#define AF_COUNTER_MAX      20  /* persistence cap                               */
#define AF_INIT_WINDOWS      9  /* startup windows used to seed the baseline     */
#define AF_I_ZERO          512  /* ADC counts at 0 A (2.5 V sensor offset)       */

/* trip codes returned by af_process_window() */
#define AF_NO_TRIP   0
#define AF_TRIP_SLOW 1  /* sustained arc signature alone                  */
#define AF_TRIP_FAST 2  /* arc signature corroborated by a current step   */

typedef struct {
  int32_t  baseline;   /* Q8: HF-envelope quiet level                     */
  int32_t  base_dev;   /* Q8: typical quiet-level deviation               */
  int32_t  asd_ema;    /* Q4: inter-window amplitude instability          */
  int16_t  hf_prev;
  int16_t  i_prev;
  uint8_t  counter;    /* persistence                                     */
  uint16_t step_age;   /* windows since last significant current step     */
  uint8_t  init_wins;
  uint8_t  tripped;
} af_state_t;

static inline void af_init(af_state_t *st) {
  st->baseline = 0;
  st->base_dev = 0;
  st->asd_ema  = 0;
  st->hf_prev  = -1;
  st->i_prev   = -1;
  st->counter  = 0;
  st->step_age = 999;
  st->init_wins = 0;
  st->tripped  = 0;
}

/* Process one completed window (AF_WIN samples of each channel).
 * Returns AF_TRIP_* on the window that first crosses the trip criterion;
 * AF_NO_TRIP otherwise.  st->tripped latches. */
static inline uint8_t af_process_window(af_state_t *st,
                                        const uint16_t *hf,
                                        const uint16_t *lf) {
  uint32_t sum = 0;
  uint8_t k;
  for (k = 0; k < AF_WIN; k++) sum += hf[k];
  int16_t hf_mean = (int16_t)(sum / AF_WIN);

  uint32_t sad = 0;
  for (k = 0; k < AF_WIN; k++) {
    int16_t d = (int16_t)hf[k] - hf_mean;
    sad += (uint16_t)(d < 0 ? -d : d);
  }
  int16_t hf_mad = (int16_t)(sad / AF_WIN);

  sum = 0;
  for (k = 0; k < AF_WIN; k++) sum += lf[k];
  int16_t i_mean = (int16_t)(sum / AF_WIN);

  /* ---- startup: seed baseline from the first AF_INIT_WINDOWS windows ---- */
  if (st->init_wins < AF_INIT_WINDOWS) {
    if (st->baseline == 0) {
      st->baseline = (int32_t)hf_mean << 8;
      st->base_dev = 4L << 8;
    } else {
      st->baseline += (((int32_t)hf_mean << 8) - st->baseline) >> 2;
    }
    st->i_prev  = i_mean;
    st->hf_prev = hf_mean;
    st->init_wins++;
    return AF_NO_TRIP;
  }

  int16_t base = (int16_t)(st->baseline >> 8);
  int16_t dev  = (int16_t)(st->base_dev >> 8);

  /* ---- (A) energy: sustained above-baseline HF noise ---- */
  int16_t margin = (int16_t)(AF_K_REL_NUM * dev);
  if (margin < AF_ABS_FLOOR) margin = AF_ABS_FLOOR;
  uint8_t energy = hf_mean > (int16_t)(base + margin);

  /* ---- (B) continuity: envelope held up within the window ---- */
  int16_t hf_ac = (int16_t)(hf_mean - base);
  if (hf_ac < 1) hf_ac = 1;
  uint8_t continuity = ((int32_t)hf_mad * AF_CONT_NUM) < hf_ac;

  /* ---- (C) chaos: window-to-window amplitude instability ---- */
  int16_t asd = (int16_t)(hf_mean - st->hf_prev);
  if (asd < 0) asd = -asd;
  st->hf_prev = hf_mean;
  st->asd_ema += (((int32_t)asd << 4) - st->asd_ema) >> AF_ASD_ALPHA_SHIFT;
  uint8_t chaos = ((st->asd_ema >> 4) * AF_CHAOS_DEN) > hf_ac;

  uint8_t suspect = energy && continuity && chaos;

  /* ---- current-step corroboration (series drop or parallel jump) ---- */
  if (st->i_prev >= 0) {
    int16_t di = (int16_t)(i_mean - st->i_prev);
    if (di < 0) di = -di;
    int16_t load = (int16_t)(st->i_prev - AF_I_ZERO);
    if (load < 0) load = -load;
    int16_t step_thr = (int16_t)(((int32_t)load * AF_STEP_FRAC_NUM) >> 6);
    if (step_thr < AF_STEP_MIN) step_thr = AF_STEP_MIN;
    if (di > step_thr) st->step_age = 0;
  }
  st->i_prev = i_mean;

  /* ---- baseline adaptation: learn only from quiet windows so an arc
   *      cannot teach the detector that arcing is normal ---- */
  if (!energy) {
    st->baseline += (((int32_t)hf_mean << 8) - st->baseline) >> AF_BASE_ALPHA_SHIFT;
    int16_t err = (int16_t)(hf_mean - (int16_t)(st->baseline >> 8));
    if (err < 0) err = -err;
    st->base_dev += (((int32_t)err << 8) - st->base_dev) >> AF_DEV_ALPHA_SHIFT;
    if (st->base_dev < (1L << 8)) st->base_dev = 1L << 8;
  }

  /* ---- persistence and trip decision ---- */
  if (suspect) { if (st->counter < AF_COUNTER_MAX) st->counter++; }
  else         { if (st->counter > 0) st->counter--; }

  uint8_t trip = AF_NO_TRIP;
  if (!st->tripped) {
    if (st->counter >= AF_N_SLOW) {
      trip = AF_TRIP_SLOW;
    } else if (st->counter >= AF_N_FAST && st->step_age <= AF_STEP_MEMORY) {
      trip = AF_TRIP_FAST;
    }
    if (trip != AF_NO_TRIP) st->tripped = 1;
  }
  if (st->step_age < 999) st->step_age++;

  return trip;
}

#endif /* DETECTOR_CORE_H */
