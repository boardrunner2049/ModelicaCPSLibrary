/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/
#include "SIM_BMP280.h"

#include <SITL/SITL.h>
#include <stdio.h>
#include <stdlib.h>

using namespace SITL;

namespace {
// Optional wire evidence, using the same Modelica state that supplies readback.
void trace_transaction(const char *operation, uint8_t reg, uint16_t length,
                       int value, uint32_t issued, const double response[20])
{
    static FILE *trace;
    static bool opened;
    if (!opened) {
        opened = true;
        const char *path = getenv("CPV_BMP280_TRACE");
        if (path != nullptr && path[0] != '\0') {
            trace = fopen(path, "w");
            if (trace == nullptr) {
                AP_HAL::panic("CPV_BMP280_TRACE: cannot write %s", path);
            }
            fprintf(trace, "time_us,operation,register,length,value,issued_id,processed_id,status,ctrl,config,raw_pressure,raw_temperature\n");
        }
    }
    if (trace != nullptr) {
        fprintf(trace, "%llu,%s,%u,%u,%d,%u,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",
                (unsigned long long)AP_HAL::micros64(), operation, unsigned(reg),
                unsigned(length), value, unsigned(issued), response[0], response[1],
                response[2], response[3], response[4], response[5]);
        fflush(trace);
    }
}
}

BMP280::Command BMP280::pending[8] {};
uint8_t BMP280::head, BMP280::count;
uint32_t BMP280::issued_sequence;

bool BMP280::next_command(double processed_sequence, double command[3])
{
    if (count != 0 && uint32_t(processed_sequence) == pending[head].sequence) {
        head = (head + 1) % ARRAY_SIZE(pending);
        count--;
    }
    if (count == 0) {
        return false;
    }
    const auto &next = pending[head];
    command[0] = next.sequence;
    command[1] = next.reg;
    command[2] = next.value;
    return true;
}

int BMP280::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    const auto &device = AP::sitl()->state.model_sensors.baro.bmp280;
    if (!device.present) {
        return -1; // No device on this bus when the composition omits BMP280.
    }
    if (data->nmsgs == 1 && data->msgs[0].flags == 0 && data->msgs[0].len == 2) {
        const uint8_t reg = data->msgs[0].buf[0];
        if (reg != 0xF4 && reg != 0xF5) {
            AP_HAL::panic("Modelica BMP280: unmodeled write to register 0x%02x", unsigned(reg));
        }
        if (count == ARRAY_SIZE(pending) || issued_sequence == INT32_MAX) {
            AP_HAL::panic("Modelica BMP280: command queue or sequence exhausted");
        }
        auto &next = pending[(head + count) % ARRAY_SIZE(pending)];
        next = {++issued_sequence, reg, data->msgs[0].buf[1]};
        count++;
        trace_transaction("write", reg, 1, next.value, next.sequence, device.response);
        // Bus acceptance does not claim that the receiver applied the write.
        return 0;
    }
    if (data->nmsgs != 2 || data->msgs[0].flags != 0 || data->msgs[0].len != 1 ||
        data->msgs[1].flags != I2C_M_RD) {
        AP_HAL::panic("Modelica BMP280: unmodeled I2C transaction");
    }

    // Response: ID,status,CTRL,CONFIG,rawP,rawT,chipID,T1,T2,T3,P1..P9,clock.
    const auto &r = device.response;
    const uint8_t base = data->msgs[0].buf[0];
    for (uint16_t i = 0; i < data->msgs[1].len; i++) {
        const uint16_t reg = uint16_t(base) + i;
        uint8_t value;
        if (reg >= 0x88 && reg <= 0x9F) {
            const uint16_t trim = uint16_t(int32_t(r[7 + (reg - 0x88) / 2]));
            value = uint8_t(trim >> (8 * ((reg - 0x88) % 2)));
        } else if (reg == 0xD0) {
            value = uint8_t(r[6]);
        } else if (reg == 0xF4 || reg == 0xF5) {
            value = uint8_t(r[reg == 0xF4 ? 2 : 3]);
        } else if (reg >= 0xF7 && reg <= 0xFC) {
            const uint32_t raw = uint32_t(r[reg < 0xFA ? 4 : 5]) << 4;
            value = uint8_t(raw >> (8 * (2 - ((reg - 0xF7) % 3))));
        } else {
            AP_HAL::panic("Modelica BMP280: unmodeled read from register 0x%02x", unsigned(reg));
        }
        data->msgs[1].buf[i] = value;
    }
    trace_transaction("read", base, data->msgs[1].len,
                      data->msgs[1].len == 1 ? data->msgs[1].buf[0] : -1,
                      issued_sequence, device.response);
    return 0;
}
