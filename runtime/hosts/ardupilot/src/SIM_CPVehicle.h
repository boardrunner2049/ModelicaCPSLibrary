/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  CP Models vehicle FMU as the SITL physics model (prototype).

  The vehicle model is linked into this program through the CP Models
  vehicle runtime (libcpvehicle) and replaces SITL's built-in vehicle
  physics; the autopilot and SITL's sensor simulation are unchanged. No
  descriptor file, shared library or socket is used at run time.

    --model cpvehicle:NAME

  NAME is a model linked into libcpvehicle: prepared with
  `python -m cp_vehicle_interface prepare MODEL.fmu OUT_DIR --shim NAME`
  and built with `cmake -DCPV_MODELS="OUT_DIR/shim;..."`.
  CPV_CHANNEL_MAP (optional) lists, for each FMU pwm element in order, the
  SITL servo output that drives it, e.g. "0,1,2,3"; the default maps
  servo k to element k+1.

  Built only with ./waf configure --cpvehicle=PATH (AP_SIM_CPVEHICLE_ENABLED).
*/
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SIM_CPVEHICLE_ENABLED
#define AP_SIM_CPVEHICLE_ENABLED 0
#endif

#if AP_SIM_CPVEHICLE_ENABLED

#include "SIM_Aircraft.h"

#include <cpvehicle/cpvehicle.h>
#include <cpvehicle/cpvehicle_model.h>

namespace SITL {

class CPVehicle : public Aircraft {
public:
    CPVehicle(const char *frame_str);
    ~CPVehicle();

    void update(const struct sitl_input &input) override;

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW CPVehicle(frame_str);
    }

private:
    void start();
    void read_model_sensors();
    void fail(const char *what, cpv_status_t status) const;

    static constexpr uint8_t max_channels = 32;  // sitl_input.servos
    cpv_vehicle_t *vehicle;
    bool started;
    bool model_mag = false, model_gnss = false, model_baro = false;  // the model has these sensors
    uint64_t start_time_us;
    size_t channel_count;
    int8_t channel_source[max_channels];  // SITL servo index per FMU pwm element, -1: none
};

}  // namespace SITL

#endif  // AP_SIM_CPVEHICLE_ENABLED
