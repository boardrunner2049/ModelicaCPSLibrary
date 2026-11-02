within ModelicaCPSLibrary.Vehicles.Copter.Presets;
record CoaxialX8 "Four arms, eight rotors; no coaxial aerodynamic interaction"
  extends Geometry(nArms=4,nRotors=8,nActuators=8,
    rotorArmIndex={1,2,3,4,1,2,3,4},spinSign={1,-1,1,-1,-1,1,-1,1},
    rotorPosition_C={armMount[rotorArmIndex[i],:] + {0,0,if i<=4 then -0.025 else 0.025} for i in 1:8});
end CoaxialX8;
