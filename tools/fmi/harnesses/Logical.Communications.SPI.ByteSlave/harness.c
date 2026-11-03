/* FMI harness: Logical.Communications.SPI.ByteSlave.
 *
 * Default parameters: mode 0 (CPOL = CPHA = 0), MSB first. The harness plays
 * an ideal 1 MHz master: SCLK rises at t0 + 0.5 + i us (sampling) and falls at
 * t0 + 1 + i us (shifting); MOSI changes on falling edges. Timeline (us):
 *    1  CS asserted (cs_n low); slave loads txData 0x5A
 *    2  byte A: MOSI 0xA5, slave answers 0x5A        -> rx 165 at 9.5
 *   10  byte B back-to-back under the same CS: MOSI 0x3C; txData 0x99 is
 *       loaded at the 8th falling edge (10 us)       -> rx 60 at 17.5
 *   19  CS released                                  -> MISO Hi-Z
 *   20  CS asserted, txData 0x0F; partial byte (4 clocks of 0xFF)
 *   24.25 CS released mid-byte                       -> nothing delivered
 *   26  CS asserted; byte C at 27: MOSI 0xC3, answer 0x0F -> rx 195 at 34.5
 *   36  CS released. Step 50 ns. */
#include "fire_harness.h"

#include <math.h>

static long long ns(double t) { return llround(t * 1e9); }

typedef struct {
  long long t0, bits; /* frame start, clock cycles */
  int mosi;           /* byte sent MSB first */
} frame_t;

static const frame_t frames[] = {{2000, 8, 0xA5}, {10000, 8, 0x3C}, {20000, 4, 0xFF}, {27000, 8, 0xC3}};

static const frame_t *frame_at(long long n) {
  for (size_t k = 0; k < sizeof frames / sizeof frames[0]; ++k)
    if (n >= frames[k].t0 && n < frames[k].t0 + 1000 * frames[k].bits)
      return &frames[k];
  return NULL;
}

static double cs_n(double t) {
  long long n = ns(t);
  return !((n >= 1000 && n < 19000) || (n >= 20000 && n < 24250) || (n >= 26000 && n < 36000));
}

static double sclk(double t) {
  long long n = ns(t);
  const frame_t *f = frame_at(n);
  if (!f)
    return 0;
  return (n - f->t0) % 1000 >= 500;
}

static double mosi(double t) {
  long long n = ns(t);
  const frame_t *f = frame_at(n);
  return f ? (f->mosi >> (7 - (n - f->t0) / 1000)) & 1 : 0;
}

static double tx_data(double t) {
  long long n = ns(t);
  return n < 9750 ? 0x5A : n < 19500 ? 0x99 : 0x0F;
}

static const fh_input_t inputs[] = {
    FH_INPUT_BOOL(cs_n, FH_FUNCTION(cs_n)),
    FH_INPUT_BOOL(sclk, FH_FUNCTION(sclk)),
    FH_INPUT_BOOL(mosi, FH_FUNCTION(mosi)),
    FH_INPUT_INT(txdata, FH_FUNCTION(tx_data)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT_BOOL(miso),
    FH_OUTPUT_BOOL(misoenabled),
    FH_OUTPUT_INT(rxdata),
    FH_OUTPUT_INT(receivedcount),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Logical.Communications.SPI.ByteSlave",
      .stop_s = 38e-6,
      .step_s = 50e-9,
      .print_s = 1e-6,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
