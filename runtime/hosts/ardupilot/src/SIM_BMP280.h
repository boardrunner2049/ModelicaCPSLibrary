/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/
#pragma once

#include "SIM_I2CDevice.h"

namespace SITL {

// I2C transport for the optional Modelica BMP280. It owns no receiver settings
// or acquisition clock: reads encode Modelica's response; writes stay ordered.
class BMP280 : public I2CDevice {
public:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    // Called by CPVehicle before a model step. Retire only the acknowledged
    // command, then deliver the oldest pending write as {ID, register, byte}.
    static bool next_command(double processed_sequence, double command[3]);

private:
    struct Command {
        uint32_t sequence;
        uint8_t reg, value;
    };
    static Command pending[8];
    static uint8_t head, count;
    static uint32_t issued_sequence;
};

} // namespace SITL
