within ModelicaCPSLibrary.Examples;
model SpiTransfer "Two bytes under one CS assertion; ideal signal interface demonstration"
  Adapters.VirtualIO.SpiTransfer link(clockFrequency=1e6);
  Logical.Communications.SPI.ByteSlave device;
  output Integer receivedCommand;
equation
  link.csActive = time >= 1e-6 and time < 25e-6;
  link.requestId = if time < 2e-6 then 0 else if time < 14e-6 then 1 else 2;
  link.txData = if time < 14e-6 then 165 else 60;
  device.cs_n = link.cs_n;
  device.sclk = link.sclk;
  device.mosi = link.mosi;
  device.txData = 90;
  link.miso = if device.misoEnabled then device.miso else true;
  receivedCommand = device.rxData;
  annotation(experiment(StopTime=30e-6,Interval=0.1e-6,Tolerance=1e-9));
end SpiTransfer;
