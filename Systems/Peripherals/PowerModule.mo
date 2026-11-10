within ModelicaCPSLibrary.Systems.Peripherals;
model PowerModule "W0: the pack's terminal voltage and the load current, measured ideally"
  extends PartialPowerModule;
equation
  voltage = battery.voltage;
  current = load;
  valid = true;
end PowerModule;
