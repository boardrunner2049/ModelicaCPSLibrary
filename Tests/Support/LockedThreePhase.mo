within ModelicaCPSLibrary.Tests.Support;
model LockedThreePhase "Fixed-angle CCM averaging benchmark; zero mechanical power"
  model LockedMotor
    extends Systems.Propulsion.Motors.ThreePhasePM(shaft(inertia(w(fixed=false),phi(fixed=false))));
    Modelica.Mechanics.Rotational.Components.Fixed lock;
  equation
    connect(lock.flange,shaft.drive);
  end LockedMotor;
  extends ThreePhaseDrive(kQ=0,redeclare LockedMotor motor);
end LockedThreePhase;
