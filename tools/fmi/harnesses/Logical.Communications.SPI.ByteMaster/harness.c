/* FMI harness: Logical.Communications.SPI.ByteMaster.
 *
 * Default parameters: 1 MHz (half period 0.5 us), mode 0 (CPOL = CPHA = 0),
 * MSB first. The harness plays host and slave: it drives requestId, txData,
 * csActive and a miso line that presents a response byte MSB first, changing
 * on SCLK falling edges (t0 + 1 + i us) so it is stable at the rising-edge
 * sampling instants (t0 + 0.5 + i us). Timeline (us):
 *    2  request 1 (0xFF) with CS inactive          -> rejected, status 2
 *    4  CS asserted (idle; no request outstanding) -> nothing happens
 *    5  request 2, tx 0xA5, slave answers 0x3C     -> success at 13, rx 60
 *   15  request 3, tx 0x81, slave answers 0xFF
 *   18.25 CS dropped mid-byte                       -> abort, status 3
 *   22  CS asserted again
 *   23  request 4, tx 0x00, slave answers 0xC5     -> success at 31, rx 197
 *   33  CS released after completion               -> no abort
 * Step 50 ns, i.e. ten samples per SCLK half period. */
#include "fire_harness.h"

#include <math.h>

static long long ns(double t) { return llround(t * 1e9); }

static double request_id(double t) {
  long long n = ns(t);
  return n < 2000 ? 0 : n < 5000 ? 1 : n < 15000 ? 2 : n < 23000 ? 3 : 4;
}

static double tx_data(double t) {
  long long n = ns(t);
  return n < 5000 ? 255 : n < 15000 ? 0xA5 : n < 23000 ? 0x81 : 0x00;
}

static double cs_active(double t) {
  long long n = ns(t);
  return (n >= 4000 && n < 18250) || (n >= 22000 && n < 33000);
}

/* Bit i (MSB first) of the response byte to the request submitted at t0 us,
 * presented on [t0 + i, t0 + i + 1) us. Idle line high (pull-up). */
static double miso_line(double t) {
  static const struct { long long t0; int byte; } frames[] = {
      {5000, 0x3C}, {15000, 0xFF}, {23000, 0xC5}};
  long long n = ns(t);
  for (size_t k = 0; k < sizeof frames / sizeof frames[0]; ++k) {
    long long dt = n - frames[k].t0;
    if (dt >= 0 && dt < 8000)
      return (frames[k].byte >> (7 - dt / 1000)) & 1;
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
      .title = "Logical.Communications.SPI.ByteMaster",
      .stop_s = 36e-6,
      .step_s = 50e-9,
      .print_s = 1e-6,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
