within ModelicaCPSLibrary.Systems.Peripherals;
model Beacon "B0: position of the mounted tag in the beacon frame (NED at the origin), truth, continuous"
  extends PartialPeripheral;
  output Real position[3](each unit="m") "NED";
  output Boolean valid = true;
  output Real validity = 1 "1 if valid, else 0";
equation
  position = mountPosition_w;
end Beacon;
