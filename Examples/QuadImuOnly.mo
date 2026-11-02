within ModelicaCPSLibrary.Examples;

model QuadImuOnly "Direct IMU composition for a sensor experiment, without SensorSuite"
  Vehicles.Copter.MultirotorPlant vehicle;
  Systems.Sensing.IMU.Sensor imu(
    r_b=vehicle.geometry.imuPosition_C-vehicle.chassis.cg_C,
    R_bs=vehicle.geometry.R_bImu);
  output Real acceleration[3] "Specific force in IMU axes [m/s2]";
  output Real gyro[3] "Angular velocity in IMU axes [rad/s]";
  output Interfaces.VehicleState truth;
  Physical.Electrical.Drives.ESC.PassThrough escs[vehicle.geometry.nRotors];
  Systems.Propulsion.Motors.FirstOrderSpeed motors[vehicle.geometry.nRotors](each omegaMax=1000,each tau=0.03,
    omega_start={sqrt(vehicle.chassis.mass*9.80665/(vehicle.geometry.nRotors*rotors[i].kT)) for i in 1:vehicle.geometry.nRotors});
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade rotors[vehicle.geometry.nRotors](
    each kT=1e-5,each kQ=1.5e-7,spinSign=vehicle.geometry.spinSign);
equation
  rotors.omega=motors.omega;
  rotors.airVelocity_r=vehicle.rotorAirVelocity_r;
  rotors.omegaBody_r=vehicle.rotorOmegaBody_r;
  vehicle.rotorForce_r=rotors.force_r;
  vehicle.rotorMoment_r=rotors.moment_r;
  motors.demand=escs.normalizedCommand;
  escs.escCommand={motors[i].omega_start/motors[i].omegaMax
    for i in 1:vehicle.geometry.nRotors};
  imu.R_wb=vehicle.body.R_wb;
  imu.a_w=vehicle.body.a_w;
  imu.gravity_w=vehicle.gravity_w;
  imu.omega_b=vehicle.body.omega_b;
  imu.alpha_b=vehicle.body.alpha_b;
  acceleration=imu.acceleration;
  gyro=imu.gyro;
  truth=vehicle.truth;
  when time>=0.01 then
    assert(max(abs(acceleration-{0,0,9.80665}))<1e-8 and max(abs(gyro))<1e-8,
      "A directly composed IMU must preserve the hover measurement contract");
  end when;
  annotation(experiment(StopTime=1,Tolerance=1e-8,Interval=0.0025));
end QuadImuOnly;
