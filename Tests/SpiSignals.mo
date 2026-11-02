within ModelicaCPSLibrary.Tests;
model SpiSignals "All four SPI modes and both bit orders; repeated data and continuous CS"
  parameter Boolean cpol[8] = {false,false,true,true,false,false,true,true};
  parameter Boolean cpha[8] = {false,true,false,true,false,true,false,true};
  parameter Boolean msbFirst[8] = {true,true,true,true,false,false,false,false};
  Adapters.VirtualIO.SpiTransfer master[8](each clockFrequency=1e6,cpol=cpol,cpha=cpha,msbFirst=msbFirst);
  Logical.Communications.SPI.ByteSlave slave[8](cpol=cpol,cpha=cpha,msbFirst=msbFirst);
  Integer request;
  Integer data;
equation
  request = if time < 2e-6 then 0 else if time < 14e-6 then 1 else if time < 26e-6 then 2
    else if time < 38e-6 then 3 else if time < 50e-6 then 4 else 5;
  data = if time < 26e-6 then 150 else if time < 38e-6 then 53 else if time < 50e-6 then 0 else 255;
  for j in 1:8 loop
    master[j].csActive = time >= 1e-6 and time < 65e-6;
    master[j].requestId = request;
    master[j].txData = data;
    slave[j].cs_n = master[j].cs_n;
    slave[j].sclk = master[j].sclk;
    slave[j].mosi = master[j].mosi;
    slave[j].txData = 105;
    master[j].miso = if slave[j].misoEnabled then slave[j].miso else true;
    when {time >= 11e-6,time >= 23e-6,time >= 35e-6,time >= 47e-6,time >= 59e-6} then
      assert(master[j].completedId == request and master[j].status == 1 and not master[j].busy,
        "SPI request did not complete successfully");
      assert(master[j].rxData == 105,"SPI MISO byte/bit order mismatch");
      assert(slave[j].rxData == data and slave[j].receivedCount == request,
        "SPI MOSI byte mismatch or repeated-byte event lost");
      assert(master[j].sclk == cpol[j],"SPI clock did not return to idle polarity");
    end when;
    when time >= 66e-6 then
      assert(not slave[j].misoEnabled and slave[j].receivedCount == 5,"SPI CS release/byte count mismatch");
    end when;
  end for;
  annotation(experiment(StopTime=70e-6,Interval=0.1e-6,Tolerance=1e-9));
end SpiSignals;
