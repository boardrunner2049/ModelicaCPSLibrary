// Harness wrapper for Adapters.VirtualIO.SpiTransfer.
//
// The adapter exports on its own, but Rumoca exports msbFirst as a fixed
// calculatedParameter (cpol/cpha stay tunable), so a world.toml override
// cannot select LSB-first order. This wrapper changes nothing but the
// parameters: 2 MHz, SPI mode 3 (CPOL = 1, CPHA = 1), LSB first, the
// combination the plain ByteMaster harness (1 MHz, mode 0, MSB first) does
// not cover.
model FireHarness_SpiTransfer
  extends ModelicaCPSLibrary.Adapters.VirtualIO.SpiTransfer(
    clockFrequency = 2e6, cpol = true, cpha = true, msbFirst = false);
end FireHarness_SpiTransfer;
