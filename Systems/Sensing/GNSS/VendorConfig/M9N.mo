within ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig;
model M9N "M9N SPG 4.04 receiver configuration over measured GNSS values"
  extends ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.PartialModel;

  // These parameters are the initial settings. Receiver commands change live state below.
  parameter Real samplePeriod(unit="s")=0.2;
  parameter Real pdopLimit=3 "CFG-NAVSPG-OUTFIL_PDOP, in DOP units";
  parameter Real staticHoldSpeedThreshold(unit="m/s")=0 "CFG-MOT-GNSSSPEED_THRS; 0 disables static hold";
  parameter Real staticHoldMaxDistance(unit="m")=0 "CFG-MOT-GNSSDIST_THRS; choose a positive distance when enabled";

  // Scenario assumption: satellite geometry is not calculated in this subset.
  parameter Real reportedPdop=2 "Controlled scenario, not a satellite-geometry solution";
  output Real navPvt[14] "Sample bookkeeping and NAV-PVT fields, in the order assigned below";
  input Real configCommand[20]=zeros(20) "Decoded request; layout in docs/m9n.md";
  output Real configResponse[20](each start=0,each fixed=true) "Completed request and readback; layout in docs/m9n.md";
protected
  // Initial parameters stay fixed; only accepted receiver commands change these values.
  discrete Real liveDynamicModel(start=0,fixed=true);
  discrete Real liveMinElevation(start=5,fixed=true);
  discrete Real livePdopLimit(fixed=false);
  discrete Real liveStaticHoldSpeedThreshold(fixed=false);
  discrete Real liveStaticHoldMaxDistance(fixed=false);
  discrete Real requestId(start=0,fixed=true) "Last valid processed ID; zero only before the first command";
  discrete Integer requestMask(start=0,fixed=true);
  Boolean setDynamicModel(start=false,fixed=true), setMinElevation(start=false,fixed=true);
  Boolean setPositionMask(start=false,fixed=true), setStaticHold(start=false,fixed=true);
  Boolean supportedWrite(start=false,fixed=true);
  ModelicaCPSLibrary.Systems.Sensing.SampledDelivery delivery(
    n=18,samplePeriod=samplePeriod,latencySamples=0);
  Real receiverPosition[3](each unit="m") "Antenna position in local North/East/Down coordinates";
  Real receiverVelocity[3](each unit="m/s") "Antenna velocity in local North/East/Down coordinates";
  Boolean solutionAccepted "Whether the solution passes this subset's PDOP check";
  Real gpsTimeOfWeekMilliseconds "GPS time of week, rounded to milliseconds";
  Real groundSpeed(unit="m/s");
  Real course(unit="deg");
  constant Integer DYNAMIC=0, WAIT=1, STATIC=2;
  discrete Integer holdState(start=DYNAMIC,fixed=true);
  discrete Real heldPosition[3](each start=0,each fixed=true,each unit="m");
  discrete Real previousVelocity[3](each start=0,each fixed=true,each unit="m/s");
  discrete Boolean havePreviousVelocity(start=false,fixed=true);
  discrete Real estimatedAcceleration(unit="m/s2",start=0,fixed=true);
  Real estimatedSpeed(unit="m/s");
  // Figure 3 factors depend on the same speed-accuracy fixture sent in NAV-PVT.
  final parameter Real speedAccuracy(unit="m/s")=0.1;
  Real entryAccelerationFactor(unit="1/s")=if speedAccuracy>2*liveStaticHoldSpeedThreshold then 0.6 else 0.4;
  Real exitAccelerationFactor(unit="1/s")=if speedAccuracy>2*liveStaticHoldSpeedThreshold then 1 else 0.7;
  Real exitSpeedFactor=if speedAccuracy>2*liveStaticHoldSpeedThreshold then 3 else 2;
initial equation
  // Read initial settings during initialization, including pre-init FMU overrides.
  pre(livePdopLimit)=pdopLimit;
  pre(liveStaticHoldSpeedThreshold)=staticHoldSpeedThreshold;
  pre(liveStaticHoldMaxDistance)=staticHoldMaxDistance;
  assert(samplePeriod>=0.04,"M9N navigation rate must not exceed 25 Hz");
  assert(reportedPdop>=0 and reportedPdop<=655.35,
    "M9N scenario PDOP must fit the NAV-PVT U2 field scaled by 0.01");
  assert(pdopLimit>=0.1 and pdopLimit<=6553.5 and
    abs(10*pdopLimit-floor(10*pdopLimit+0.5))<1e-8,
    "M9N PDOP limit requires a positive U2 value scaled by 0.1, in [0.1,6553.5]");
  assert(staticHoldSpeedThreshold>=0 and staticHoldSpeedThreshold<=2.55 and
    abs(100*staticHoldSpeedThreshold-floor(100*staticHoldSpeedThreshold+0.5))<1e-8,
    "M9N static-hold speed requires U1 scaled by 0.01 m/s, in [0,2.55]");
  assert(staticHoldMaxDistance>=0 and staticHoldMaxDistance<=65535 and
    staticHoldMaxDistance<=floor(staticHoldMaxDistance),
    "M9N static-hold distance requires an integer U2 value in [0,65535] m");
  assert(staticHoldSpeedThreshold<=0 or staticHoldMaxDistance>0,
    "M9N enabled static hold requires a positive distance; firmware-default distance is unmodeled");
equation
  // Check each discrete ID directly. Zero means idle only before the first command.
  assert(noEvent((configCommand[1]>=1 and configCommand[1]<=2147483647 and
    abs(configCommand[1]-floor(configCommand[1]))<=0) or
    (abs(configCommand[1])<=0 and abs(requestId)<=0)),
    "M9N command ID must be an integer in [1,2147483647]; zero is the initial idle value");
  // Inputs already include antenna mounting and physical biases.
  estimatedSpeed=sqrt(measuredVelocity*measuredVelocity);

  // Approximate the receiver's unpublished motion estimates using raw 3D P/V.
  // The first sample supplies history; two later quiet samples enter STATIC.
  when sample(0,samplePeriod) then
    estimatedAcceleration=if pre(havePreviousVelocity) then
      sqrt((measuredVelocity-pre(previousVelocity))*(measuredVelocity-pre(previousVelocity)))/samplePeriod else 0;
    previousVelocity=measuredVelocity;
    havePreviousVelocity=true;
    holdState=if liveStaticHoldSpeedThreshold<=0 or not solutionAccepted or not pre(havePreviousVelocity) then DYNAMIC
      elseif pre(holdState)==STATIC then
        if estimatedAcceleration>exitAccelerationFactor*liveStaticHoldSpeedThreshold or
          estimatedSpeed>exitSpeedFactor*liveStaticHoldSpeedThreshold or
          sqrt((measuredPosition-pre(heldPosition))*(measuredPosition-pre(heldPosition)))>liveStaticHoldMaxDistance
        then DYNAMIC else STATIC
      elseif estimatedAcceleration<entryAccelerationFactor*liveStaticHoldSpeedThreshold and
        estimatedSpeed<liveStaticHoldSpeedThreshold then
        if pre(holdState)==WAIT then STATIC else WAIT
      else DYNAMIC;
    heldPosition=if holdState==STATIC and pre(holdState)<>STATIC then measuredPosition else pre(heldPosition);
  end when;

  // Select the updated state before this acquisition is packed by delivery.
  receiverPosition=if holdState==STATIC then heldPosition else measuredPosition;
  receiverVelocity=if holdState==STATIC then zeros(3) else measuredVelocity;
  groundSpeed=sqrt(receiverVelocity[1]^2+receiverVelocity[2]^2);
  // Use this acquisition's speed, including the sample that releases static hold.
  course=if noEvent(groundSpeed>0) then
    mod(atan2(receiverVelocity[2],receiverVelocity[1])*180/Modelica.Constants.pi,360) else 0;
  gpsTimeOfWeekMilliseconds=
    noEvent(mod(floor(1000*time+0.5),604800000));

  // Equality rejects. Invalidity also releases static hold; fixType stays 3D.
  solutionAccepted=reportedPdop<livePdopLimit;

  // Acquire all fields together. Quality constants are fixtures, not calibration.
  delivery.u[1:3]=receiverPosition;           // Local NED, metres.
  delivery.u[4:6]=receiverVelocity;           // Local NED, metres/second.
  delivery.u[7]=gpsTimeOfWeekMilliseconds;    // iTOW wraps at one GPS week.
  delivery.u[8]=3;                           // Receiver fixType: 3D solution.
  delivery.u[9]=if solutionAccepted then 1 else 0; // Only flag bit 0: gnssFixOK.
  delivery.u[10]=10;                         // Number of satellites used.
  delivery.u[11]=1;                          // Horizontal accuracy, metres.
  delivery.u[12]=1;                          // Vertical accuracy, metres.
  delivery.u[13]=speedAccuracy;              // Speed accuracy, metres/second.
  delivery.u[14]=reportedPdop;               // Dimensionless PDOP.
  delivery.u[15]=groundSpeed;                // Horizontal speed, metres/second.
  delivery.u[16]=course;                     // Course clockwise from north, degrees.
  delivery.u[17]=180;                        // Course accuracy, degrees.
  delivery.u[18]=0;                          // Ellipsoid minus MSL height, metres.
  delivery.acquisitionValid=solutionAccepted;
  delivery.acquisitionStatus=3;

  // These held outputs and metadata all describe the same acquisition.
  position=delivery.y[1:3];
  velocity=delivery.y[4:6];
  navPvt[1]=delivery.sequence;                // Bookkeeping; not a UBX field.
  navPvt[2]=delivery.sampleTime;              // Acquisition time, simulation seconds.
  navPvt[3:14]=delivery.y[7:18];              // Same field order and units as delivery.u above.
algorithm
  // Read one decoded request in order, then publish its completed response.
  // NAV5 applies only the selected groups. Other unmodeled requests return
  // status 4 to the harness; that is not a receiver ACK or NAK.
  when abs(configCommand[1]-pre(requestId))>0 then
    requestMask:=integer(configCommand[5]);
    setDynamicModel:=mod(requestMask,2)==1;
    setMinElevation:=mod(div(requestMask,2),2)==1;
    setPositionMask:=mod(div(requestMask,16),2)==1;
    setStaticHold:=mod(div(requestMask,64),2)==1;
    // Validate all fields at this command event, rather than separate threshold events.
    supportedWrite:=integer(configCommand[2])==6 and integer(configCommand[3])==36 and integer(configCommand[4])==1 and
    requestMask==mod(requestMask,4)+16*mod(div(requestMask,16),2)+64*mod(div(requestMask,64),2) and
    (not setDynamicModel or (abs(configCommand[6])<1e-8 or
      (configCommand[6]>=2 and configCommand[6]<=9 and abs(floor(configCommand[6])-configCommand[6])<1e-8))) and
    (not setMinElevation or (configCommand[10]>=-128 and configCommand[10]<=127 and
      abs(floor(configCommand[10])-configCommand[10])<1e-8)) and
    (not setPositionMask or (configCommand[11]>0 and configCommand[11]<=6553.5 and
      abs(10*configCommand[11]-floor(10*configCommand[11]+0.5))<1e-8 and abs(configCommand[13]-100)<1e-8)) and
    (not setStaticHold or (configCommand[15]>=0 and configCommand[15]<=2.55 and
      abs(100*configCommand[15]-floor(100*configCommand[15]+0.5))<1e-8 and
      configCommand[19]>=0 and configCommand[19]<=65535 and abs(floor(configCommand[19])-configCommand[19])<1e-8 and
      (configCommand[15]<=0 or configCommand[19]>0)));
    liveDynamicModel:=if supportedWrite and setDynamicModel then configCommand[6] else pre(liveDynamicModel);
    liveMinElevation:=if supportedWrite and setMinElevation then configCommand[10] else pre(liveMinElevation);
    livePdopLimit:=if supportedWrite and setPositionMask then configCommand[11] else pre(livePdopLimit);
    liveStaticHoldSpeedThreshold:=if supportedWrite and setStaticHold then configCommand[15] else pre(liveStaticHoldSpeedThreshold);
    liveStaticHoldMaxDistance:=if supportedWrite and setStaticHold then configCommand[19] else pre(liveStaticHoldMaxDistance);
    // Complete NAV5 fields, in semantic units. Unchanged fields are documented
    // factory fixtures; dynamic-model filtering and satellite selection are unmodeled.
    configResponse:=if integer(configCommand[2])==10 and integer(configCommand[3])==4 and integer(configCommand[4])==0 then
      // Fixed product identity: hardware digits, CORE version, protocol version,
      // and M9N module family. The encoder marks the build as emulated.
      {configCommand[1],10,4,1,190000,4,4,32,1,9,0,0,0,0,0,0,0,0,0,0}
    elseif integer(configCommand[2])==6 and integer(configCommand[3])==0 and integer(configCommand[4])==0 then
      // Current UART1 fixture: 230400 baud, 8N1, UBX input/output only.
      // Baud timing and port changes are unmodeled; reserved bits stay zero.
      {configCommand[1],6,0,1,1,0,2240,230400,1,1,0,0,0,0,0,0,0,0,0,0}
    else {
      configCommand[1], configCommand[2], configCommand[3],
      if integer(configCommand[2])==6 and integer(configCommand[3])==36 and integer(configCommand[4])==0 then 1
        elseif supportedWrite then 2 else 4,
      0,                                          // Readback mask convention.
      liveDynamicModel,
      3,                                          // Automatic 2D/3D.
      0,                                          // Fixed altitude, metres.
      1,                                          // Fixed altitude variance, m2.
      liveMinElevation,
      livePdopLimit,
      25,                                         // Time DOP limit.
      100,                                        // Position accuracy limit, m.
      350,                                        // Time accuracy limit, m.
      liveStaticHoldSpeedThreshold,
      60,                                         // DGNSS timeout, seconds.
      0, 0,                                       // CNO thresholds.
      liveStaticHoldMaxDistance,
      0};                                         // Automatic UTC standard.
    // Preserve the last valid ID so a later zero cannot look like startup idle.
    requestId:=if configCommand[1]>=1 and configCommand[1]<=2147483647 and
      abs(configCommand[1]-floor(configCommand[1]))<=0 then configCommand[1] else pre(requestId);
  end when;
end M9N;
