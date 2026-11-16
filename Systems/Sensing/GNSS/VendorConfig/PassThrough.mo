within ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig;
model PassThrough "Forward physics measurements without sampling or added state"
  extends PartialModel;
equation
  position=measuredPosition;
  velocity=measuredVelocity;
end PassThrough;
