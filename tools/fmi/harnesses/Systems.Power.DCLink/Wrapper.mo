// Harness wrapper for ModelicaCPSLibrary.Systems.Power.DCLink.
// DCLink has no defaults for sourceResistance, dcLinkCapacitance and
// voltage_start, and its p/n bus pins are acausal; exported alone it is
// refused ("dcLinkCapacitance has no static binding"). The wrapper uses the
// values of the library's own three-phase bench (Rs = 20 mOhm, C = 2 mF,
// starting at 16 V; time constant Rs*C = 40 us) and loads the bus with an
// ideal current sink whose current is a harness input (negative = current
// returned to the bus, e.g. regenerative braking).
// The bus-voltage assert below is a sanity guard and also a workaround: with
// bin/rumoca of 2026-11-03 a model whose only assertion is the MSL Resistor's
// parameter-only temperature assert (a plain R-C or R-L circuit) is refused by
// the FMI C profile ("only unscheduled assertions without relation memory are
// supported"); adding any assertion on a continuous variable lets it export.
model FireHarness_DCLink
  input Real sourceVoltage(unit = "V", start = 16) "Upstream ideal source voltage";
  input Real loadCurrent(unit = "A", start = 0) "Current drawn from the bus by the load";
  ModelicaCPSLibrary.Systems.Power.DCLink bus(sourceResistance = 0.02, dcLinkCapacitance = 0.002, voltage_start = 16);
  Modelica.Electrical.Analog.Sources.SignalCurrent load "Ideal current sink p -> n";
  output Real voltage(unit = "V") = bus.voltage;
  output Real sourceCurrent(unit = "A") = bus.sourceCurrent;
  output Real sourcePower(unit = "W") = bus.sourcePower;
  output Real storedEnergy(unit = "J") = bus.storedEnergy;
equation
  bus.sourceVoltage = sourceVoltage;
  load.i = loadCurrent;
  connect(load.p, bus.p);
  connect(load.n, bus.n);
  assert(bus.voltage > 0, "DC bus voltage collapsed");
end FireHarness_DCLink;
