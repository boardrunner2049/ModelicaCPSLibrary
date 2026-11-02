within ModelicaCPSLibrary.Tests;
model NedFrdContract "Same physical motion in old ENU/FLU and new NED/FRD bases"
  constant Real W[3,3]=[0,1,0;1,0,0;0,0,-1];
  constant Real B[3,3]=diagonal({1,-1,-1});
  parameter Real J[3,3]=[2,0.3,-0.2;0.3,3,0.4;-0.2,0.4,4];
  parameter Real qOld[4]=Utilities.Math.euler321ToQuaternion({0.3,-0.2,0.7});
  parameter Real qNew[4]=Utilities.Math.quaternionProduct(
    Utilities.Math.quaternionProduct({0,sqrt(0.5),sqrt(0.5),0},qOld),{0,-1,0,0});
  parameter Real mountOld[3,3]=Utilities.Math.euler321ToRotationMatrix({-0.4,0.2,0.1});
  parameter Real rOld[3]={0.1,-0.2,0.3};
  Physical.Mechanical.Dynamics.RigidBody6DOF oldBody(
    mass=2,inertia=J,p_start={1,2,3},v_start={0.2,-0.1,0.3},q_start=qOld,omega_start={0.1,0.2,-0.3});
  Physical.Mechanical.Dynamics.RigidBody6DOF body(
    mass=2,inertia=B*J*B,p_start={2,1,-3},v_start={0.2,0.1,-0.3},q_start=qNew,omega_start={0.1,-0.2,0.3});
  Systems.Sensing.IMU.Sensor oldImu(r_b=rOld,R_bs=mountOld);
  Systems.Sensing.SensorSuite sensors(rImu_b=B*rOld,R_bImu=B*mountOld,R_bMag=B*mountOld,
    rGnss_b=B*rOld,rBarometer_b=B*rOld);
  Adapters.VirtualIO.SensorValues values(geometry(R_bImu=B*mountOld,R_bMag=B*mountOld));
  Real antennaOld[3];
  Real rotorForceOld[3];
  Real rotorMomentOld[3];
  Real rotorForce[3];
  Real rotorMoment[3];
equation
  oldBody.force_b={1,2,4}; oldBody.moment_b={0.1,-0.2,0.3}; oldBody.gravity_w={0,0,-9.80665};
  body.force_b=B*oldBody.force_b; body.moment_b=B*oldBody.moment_b; body.gravity_w=W*oldBody.gravity_w;
  oldImu.R_wb=oldBody.R_wb; oldImu.a_w=oldBody.a_w; oldImu.gravity_w=oldBody.gravity_w;
  oldImu.omega_b=oldBody.omega_b; oldImu.alpha_b=oldBody.alpha_b;
  sensors.p_w=body.p_w; sensors.v_w=body.v_w; sensors.R_wb=body.R_wb;
  sensors.a_w=body.a_w; sensors.omega_b=body.omega_b; sensors.alpha_b=body.alpha_b;
  sensors.gravity_w=body.gravity_w; sensors.magneticField_w=W*{20e-6,5e-6,-45e-6};
  sensors.pressure=101325; sensors.temperature=288.15;
  values.measurements=sensors.measurements;
  antennaOld=oldBody.p_w+oldBody.R_wb*rOld;
  rotorForceOld=mountOld*{0,0,4};
  rotorMomentOld=cross(rOld,rotorForceOld)+mountOld*{0,0,-0.2};
  rotorForce=B*mountOld*{0,0,4};
  rotorMoment=cross(B*rOld,rotorForce)+B*mountOld*{0,0,-0.2};
  assert(max(abs(body.R_wb-W*oldBody.R_wb*B))<1e-7,"Full attitude transformation failed");
  assert(max(abs(body.p_w-W*oldBody.p_w))<1e-7 and max(abs(body.v_w-W*oldBody.v_w))<1e-7,"World motion transformation failed");
  assert(max(abs(body.omega_b-B*oldBody.omega_b))<1e-7,"Full inertia/angular motion transformation failed");
  assert(max(abs(sensors.measurements.acceleration-oldImu.acceleration))<1e-7,"Sensor local axes must remain unchanged");
  assert(max(abs(values.acceleration_frd-B*mountOld*oldImu.acceleration))<1e-7,"Mount must be applied exactly once");
  assert(max(abs(sensors.measurements.magneticField-transpose(mountOld)*transpose(oldBody.R_wb)*{20e-6,5e-6,-45e-6}))<1e-10,"Magnetic frame transformation failed");
  assert(max(abs(values.position_ned-W*(oldBody.p_w+oldBody.R_wb*rOld)))<1e-7,"GNSS mount or NED conversion failed");
  assert(max(abs(values.velocity_ned-W*(oldBody.v_w+oldBody.R_wb*cross(oldBody.omega_b,rOld))))<1e-7,"GNSS lever arm velocity failed");
  assert(abs(sensors.measurements.altitude-antennaOld[3])<1e-7,"Altitude must remain up-positive");
  assert(max(abs(rotorMoment-B*rotorMomentOld))<1e-12 and max(abs(rotorForce-B*rotorForceOld))<1e-12,"Hub wrench and moment arm migration failed");
  annotation(experiment(StopTime=0.1,Tolerance=1e-10,Interval=0.001));
end NedFrdContract;
