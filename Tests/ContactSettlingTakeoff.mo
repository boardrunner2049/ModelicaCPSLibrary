within ModelicaCPSLibrary.Tests;

model ContactSettlingTakeoff "Four-point quad/hexa support, sensor force, takeoff and second landing"
  Examples.QuadGroundCycle quad;
  Examples.QuadGroundCycle hexa(vehicle(
    geometry=Vehicles.Copter.Presets.HexaX(),aggregate(mass=2.2)));
  Real dissipation(unit="W");
equation
  dissipation = sum((quad.vehicle.gear.normalForce[i]-quad.vehicle.landingGear.stiffness*
    max(0,-quad.vehicle.gear.gap[i]))*
    (-quad.vehicle.gear.legs[i].tipVelocity_w[3]) for i in 1:4);
  assert(dissipation <= 1e-7,"Unilateral spring/damper contact must not generate energy");
  assert(min(quad.vehicle.gear.normalForce)>=0 and min(hexa.vehicle.gear.normalForce)>=0,
    "Contact cannot pull a departing vehicle toward the ground");
  when time >= 1.5 then
    assert(quad.contactCount==4 and hexa.contactCount==4,"Quad and hexa must both have four supporting feet");
    assert(abs(quad.altitude-(0.1-1.5*9.80665/12000))<1e-6 and
      abs(hexa.altitude-(0.1-2.2*9.80665/12000))<1e-6,"Static compression must equal mg/(4k)");
    assert(abs(quad.verticalVelocity)<1e-5 and abs(hexa.verticalVelocity)<1e-5,"Vehicles must settle before motor startup");
    assert(abs(quad.normalLoad-1.5*9.80665)<1e-4 and abs(hexa.normalLoad-2.2*9.80665)<1e-4,
      "Ground must support each vehicle's complete mass exactly once");
    assert(abs(quad.acceleration_frd[3]+9.80665)<1e-4 and abs(hexa.acceleration_frd[3]+9.80665)<1e-4,
      "Supported FRD accelerometers must report minus g");
  end when;
  when time >= 2.5 then
    assert(quad.contactCount==0 and hexa.contactCount==0 and abs(quad.normalLoad)+abs(hexa.normalLoad)<1e-12,
      "Both vehicles must release all four contacts during takeoff");
    assert(quad.altitude>0.15 and hexa.altitude>0.15 and
      quad.verticalVelocity<0 and hexa.verticalVelocity<0,"Thrust must lift the vehicles in negative NED Z");
  end when;
  when time >= 5.5 then
    assert(quad.contactCount==4 and hexa.contactCount==4 and
      abs(quad.verticalVelocity)<1e-5 and abs(hexa.verticalVelocity)<1e-5,
      "Both vehicles must land and settle again after motor cutoff");
    assert(abs(quad.altitude-(0.1-1.5*9.80665/12000))<1e-6 and
      abs(hexa.altitude-(0.1-2.2*9.80665/12000))<1e-6,"Second landing must return to static compression");
  end when;
  annotation(experiment(StopTime=6,Tolerance=1e-8,Interval=0.002));
end ContactSettlingTakeoff;
