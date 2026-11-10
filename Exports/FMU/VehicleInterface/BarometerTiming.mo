within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model BarometerTiming "barometer_timing_v1: when the barometer reading was acquired and delivered"
  output Real baro_sample_time_s(unit="s") "Acquisition time of the delivered reading, simulation time; -1 before the first [s]";
  output Real baro_delivery_time_s(unit="s") "Time the delivered reading became available, simulation time; -1 before the first [s]";
  output Real baro_sequence(unit="1") "Delivered readings (an integer); a new reading exactly when it increments";
  output Real baro_valid(unit="1") "1 once a valid reading has been delivered, else 0";
  Integer barometerSequence "Bound by the extending model to the plant's barometer sequence";
  Boolean barometerValid "Bound by the extending model to the plant's barometer validity";
equation
  baro_sequence = barometerSequence;
  baro_valid = if barometerValid then 1 else 0;
end BarometerTiming;
