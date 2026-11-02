within ModelicaCPSLibrary.Logical.Communications.SPI;
block ByteSlave "Ideal SPI byte endpoint; device-specific register decoding is external"
  parameter Boolean cpol = false;
  parameter Boolean cpha = false;
  parameter Boolean msbFirst = true;
  input Boolean cs_n;
  input Boolean sclk;
  input Boolean mosi;
  input Integer txData "Unsigned response byte, 0..255";
  output Boolean miso;
  output Boolean misoEnabled "False means high impedance; miso alone has no Hi-Z encoding";
  output Integer rxData(start=0,fixed=true) "Last fully received byte";
  output Integer receivedCount(start=0,fixed=true) "Changes for every byte, including repeated data";
protected
  discrete Integer txShift(start=0,fixed=true);
  discrete Integer txCount(start=0,fixed=true);
  discrete Integer rxShift(start=0,fixed=true);
  discrete Integer rxCount(start=0,fixed=true);
initial equation
  pre(cs_n) = cs_n;
  pre(sclk) = sclk;
  pre(mosi) = mosi;
equation
  misoEnabled = not cs_n;
  miso = misoEnabled and (if msbFirst then div(txShift,128) <> 0 else mod(txShift,2) <> 0);
algorithm
  when {initial(),change(cs_n),change(sclk)} then
    if initial() or change(cs_n) then
      if initial() then
        rxData := 0;
        receivedCount := 0;
      end if;
      txShift := 0;
      rxShift := 0;
      rxCount := 0;
      txCount := 0;
      if not cs_n then
        assert(txData >= 0 and txData <= 255,"SPI slave txData must be an unsigned byte");
        txShift := txData;
      end if;
    elseif not cs_n then
      if (not cpha and sclk <> cpol) or (cpha and sclk == cpol) then
        rxShift := if msbFirst then 2*pre(rxShift)+(if pre(mosi) then 1 else 0)
          else pre(rxShift)+(if pre(mosi) then integer(2^pre(rxCount)) else 0);
        if pre(rxCount) == 7 then
          rxData := rxShift;
          receivedCount := pre(receivedCount)+1;
          rxCount := 0;
          rxShift := 0;
        else
          rxCount := pre(rxCount)+1;
        end if;
      else
        if (cpha and pre(txCount) == 0) or (not cpha and pre(txCount) == 7) then
          assert(txData >= 0 and txData <= 255,"SPI slave txData must be an unsigned byte");
          txShift := txData;
        else
          txShift := if msbFirst then mod(2*pre(txShift),256) else div(pre(txShift),2);
        end if;
        txCount := mod(pre(txCount)+1,8);
      end if;
    end if;
  end when;
end ByteSlave;
