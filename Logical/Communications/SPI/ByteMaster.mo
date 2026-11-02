within ModelicaCPSLibrary.Logical.Communications.SPI;
block ByteMaster "One outstanding byte request; ideal CPOL/CPHA signal generation"
  parameter Modelica.Units.SI.Frequency clockFrequency = 1e6;
  parameter Boolean cpol = false;
  parameter Boolean cpha = false;
  parameter Boolean msbFirst = true;
  input Integer requestId(start=0) "Zero at initialization; change to submit a byte";
  input Integer txData "Unsigned byte, 0..255";
  input Boolean csActive "Host owns chip select across bytes";
  input Boolean miso "Sampled return line; resolve disabled slave outputs externally";
  output Boolean cs_n;
  output Boolean sclk(start=cpol,fixed=true);
  output Boolean mosi;
  output Boolean busy(start=false,fixed=true);
  output Integer completedId(start=0,fixed=true) "Last retired request, including rejected/aborted requests";
  output Integer rxData(start=0,fixed=true) "Last successfully received byte";
  output Integer status(start=0,fixed=true) "0 idle, 1 success, 2 CS inactive, 3 aborted";
protected
  discrete Modelica.Units.SI.Time nextEdge(start=Modelica.Constants.inf,fixed=true);
  discrete Integer activeId(start=0,fixed=true);
  discrete Integer edgeCount(start=0,fixed=true);
  discrete Integer txShift(start=0,fixed=true);
  discrete Integer rxShift(start=0,fixed=true);
  discrete Integer rxCount(start=0,fixed=true);
initial equation
  pre(requestId) = 0;
  pre(csActive) = csActive;
  pre(miso) = miso;
  assert(clockFrequency > 0,"SPI clockFrequency must be positive");
equation
  cs_n = not csActive;
  mosi = if msbFirst then div(txShift,128) <> 0 else mod(txShift,2) <> 0;
algorithm
  when {initial(),change(requestId),change(csActive),time >= pre(nextEdge)} then
    if initial() then
      assert(requestId == 0,"SPI requestId must start at zero; submit after initialization");
      sclk := cpol;
      busy := false;
      completedId := 0;
      rxData := 0;
      status := 0;
      nextEdge := Modelica.Constants.inf;
      activeId := 0;
      edgeCount := 0;
      txShift := 0;
      rxShift := 0;
      rxCount := 0;
    elseif change(requestId) then
      assert(not pre(busy),"SPI allows one outstanding request; wait for completedId before submitting");
      assert(requestId > pre(requestId),"SPI requestId must increase for every request");
      assert(txData >= 0 and txData <= 255,"SPI txData must be an unsigned byte");
      if not csActive then
        completedId := requestId;
        status := 2;
      else
        activeId := requestId;
        txShift := txData;
        rxShift := 0;
        rxCount := 0;
        edgeCount := 0;
        busy := true;
        status := 0;
        sclk := cpol;
        nextEdge := time+0.5/clockFrequency;
      end if;
    elseif not csActive and pre(busy) then
      busy := false;
      sclk := cpol;
      nextEdge := Modelica.Constants.inf;
      completedId := pre(activeId);
      status := 3;
    elseif pre(busy) and time >= pre(nextEdge) then
      edgeCount := pre(edgeCount)+1;
      sclk := not pre(sclk);
      if (not cpha and mod(edgeCount,2) == 1) or (cpha and mod(edgeCount,2) == 0) then
        rxShift := if msbFirst then 2*pre(rxShift)+(if pre(miso) then 1 else 0)
          else pre(rxShift)+(if pre(miso) then integer(2^pre(rxCount)) else 0);
        rxCount := pre(rxCount)+1;
      elseif (not cpha) or edgeCount > 1 then
        txShift := if msbFirst then mod(2*pre(txShift),256) else div(pre(txShift),2);
      end if;
      if edgeCount == 16 then
        rxData := rxShift;
        completedId := pre(activeId);
        status := 1;
        busy := false;
        nextEdge := Modelica.Constants.inf;
      else
        nextEdge := time+0.5/clockFrequency;
      end if;
    end if;
  end when;
end ByteMaster;
