within ModelicaCPSLibrary.Tests;
model SpiDisruption "Inactive CS rejection, partial-byte cancellation, recovery and logical line disruption"
  Adapters.VirtualIO.SpiTransfer rejected(clockFrequency=1e6);
  Adapters.VirtualIO.SpiTransfer cancelled(clockFrequency=1e6);
  Logical.Communications.SPI.ByteSlave cancelSlave;
  Adapters.VirtualIO.SpiTransfer disturbed(clockFrequency=1e6);
  Logical.Communications.SPI.ByteSlave disturbedSlave;
equation
  rejected.requestId = if time < 2e-6 then 0 else 1;
  rejected.txData = 255;
  rejected.csActive = false;
  rejected.miso = true;
  cancelled.requestId = if time < 2e-6 then 0 else if time < 12e-6 then 1 else 2;
  cancelled.txData = 53;
  cancelled.csActive = (time >= 1e-6 and time < 6e-6) or (time >= 11e-6 and time < 24e-6);
  cancelSlave.cs_n = cancelled.cs_n;
  cancelSlave.sclk = cancelled.sclk;
  cancelSlave.mosi = cancelled.mosi;
  cancelSlave.txData = 105;
  cancelled.miso = if cancelSlave.misoEnabled then cancelSlave.miso else true;
  disturbed.requestId = if time < 2e-6 then 0 else 1;
  disturbed.txData = 150;
  disturbed.csActive = time >= 1e-6 and time < 14e-6;
  disturbedSlave.cs_n = disturbed.cs_n;
  disturbedSlave.sclk = disturbed.sclk;
  // Invert the line around the first sampling edge. This is a logical fault, not a calibrated EMI model.
  disturbedSlave.mosi = disturbed.mosi <> (time >= 2.25e-6 and time < 2.75e-6);
  disturbedSlave.txData = 105;
  disturbed.miso = if disturbedSlave.misoEnabled then disturbedSlave.miso else true;
  when time >= 3e-6 then
    assert(rejected.completedId == 1 and rejected.status == 2 and not rejected.busy,
      "Inactive CS request was not rejected");
  end when;
  when time >= 7e-6 then
    assert(cancelled.completedId == 1 and cancelled.status == 3 and not cancelled.busy,
      "CS cancellation was not reported");
    assert(cancelSlave.receivedCount == 0,"Partial SPI byte was delivered");
  end when;
  when time >= 11e-6 then
    assert(disturbedSlave.rxData == 22 and disturbedSlave.receivedCount == 1,
      "MOSI line disruption did not change the sampled first bit");
    assert(disturbed.rxData == 105 and disturbed.status == 1,
      "MOSI disturbance incorrectly changed the independent MISO path");
  end when;
  when time >= 22e-6 then
    assert(cancelled.completedId == 2 and cancelled.status == 1 and cancelled.rxData == 105,
      "SPI master failed to recover after cancellation");
    assert(cancelSlave.receivedCount == 1 and cancelSlave.rxData == 53,
      "SPI slave failed to recover after cancellation");
  end when;
  annotation(experiment(StopTime=26e-6,Interval=0.05e-6,Tolerance=1e-9));
end SpiDisruption;
