/* FMI harness: Adapters.VirtualIO.SpiTransfer (extends SPI.ByteMaster).
 *
 * The adapter is the boardrunner byte-request boundary. It is run here in the
 * mode the plain ByteMaster harness does not cover, set via world.toml
 * parameters: 2 MHz (half period 0.25 us, byte = 4 us), CPOL = 1, CPHA = 1
 * (mode 3: idle high, sample on the even/rising edges at t0 + 0.5(k+1) us),
 * LSB first. The harness follows the documented host contract (set CS before
 * submitting, hold it through completion, wait for completedId) and plays a
 * mode-3 slave whose MISO bit k is presented on [t0 + 0.25 + 0.5k,
 * t0 + 0.75 + 0.5k) us, i.e. changes on falling edges. Timeline (us):
 *    1  CS asserted
 *    2  request 1, tx 0x35, slave answers 0xC5   -> success at 6, rx 197
 *    7  request 2, tx 0x80, slave answers 0x01   -> success at 11, rx 1
 *   12  CS released; 13 request 3 with CS off    -> rejected, status 2
 *   14  CS asserted; 15 request 4, tx 0xFF
 *   16.6 CS released mid-byte                    -> aborted, status 3
 * Step 25 ns (ten samples per half period). */
#include "fire_harness.h"

#include <math.h>

static long long ns(double t) { return llround(t * 1e9); }

static double request_id(double t) {
  long long n = ns(t);
  return n < 2000 ? 0 : n < 7000 ? 1 : n < 13000 ? 2 : n < 15000 ? 3 : 4;
}

static double tx_data(double t) {
  long long n = ns(t);
  return n < 7000 ? 0x35 : n < 15000 ? 0x80 : 0xFF;
}

static double cs_active(double t) {
  long long n = ns(t);
  return (n >= 1000 && n < 12000) || (n >= 14000 && n < 16600);
}

/* LSB-first response bit k on [t0 + 250 + 500k, t0 + 750 + 500k) ns; the
 * line is held at bit 0 from t0 and idles high outside frames. */
static double miso_line(double t) {
  static const struct { long long t0; int byte; } frames[] = {{2000, 0xC5}, {7000, 0x01}, {15000, 0x00}};
  long long n = ns(t);
  for (size_t k = 0; k < sizeof frames / sizeof frames[0]; ++k) {
    long long dt = n - frames[k].t0;
    if (dt >= 0 && dt < 4000) {
      long long bit = dt < 250 ? 0 : (dt - 250) / 500;
      return (frames[k].byte >> (bit > 7 ? 7 : bit)) & 1;
    }
  }
  return 1;
}

static const fh_input_t inputs[] = {
    FH_INPUT_INT(requestid, FH_FUNCTION(request_id)),
    FH_INPUT_INT(txdata, FH_FUNCTION(tx_data)),
    FH_INPUT_BOOL(csactive, FH_FUNCTION(cs_active)),
    FH_INPUT_BOOL(miso, FH_FUNCTION(miso_line)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT_BOOL(cs_n),
    FH_OUTPUT_BOOL(sclk),
    FH_OUTPUT_BOOL(mosi),
    FH_OUTPUT_BOOL(busy),
    FH_OUTPUT_INT(completedid),
    FH_OUTPUT_INT(rxdata),
    FH_OUTPUT_INT(status),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Adapters.VirtualIO.SpiTransfer",
      .stop_s = 18e-6,
      .step_s = 25e-9,
      .print_s = 1e-6,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
