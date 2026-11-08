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
  CP Models vehicle FMU as the SITL physics model (prototype); see SIM_CPVehicle.h.
*/
#include "SIM_CPVehicle.h"

#if AP_SIM_CPVEHICLE_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SITL/SITL.h>
#include <AP_Declination/AP_Declination.h>

using namespace SITL;

namespace {
// What this host requires. The model's magnetometer, GNSS and barometer are
// used when it has them (magnetometer_v1, gnss_v1, barometer_v1); otherwise
// SITL simulates that sensor from the model's truth.
const char *const required_capabilities[] = {
    "pwm_actuation_v1",
    "kinematic_state_v1",
    "imu_v1",
};
}

CPVehicle::CPVehicle(const char *frame_str) :
    Aircraft(frame_str)
{
    // The models linked into libcpvehicle (cmake -DCPV_MODELS=...).
    char linked[256] = "";
    for (size_t i = 0; i < cpv_model_count(); i++) {
        const size_t used = strlen(linked);
        snprintf(linked + used, sizeof(linked) - used, "%s%s", i ? ", " : "",
                 cpv_model_at(i)->name);
    }
    const char *colon = strchr(frame_str, ':');
    if (colon == nullptr || colon[1] == '\0') {
        AP_HAL::panic("cpvehicle: use --model cpvehicle:NAME; linked models: %s",
                      linked[0] ? linked : "none (build libcpvehicle with -DCPV_MODELS)");
    }
    const cpv_model_t *model = cpv_find_model(colon + 1);
    if (model == nullptr) {
        AP_HAL::panic("cpvehicle: no linked model '%s'; linked models: %s", colon + 1,
                      linked[0] ? linked : "none (build libcpvehicle with -DCPV_MODELS)");
    }
    cpv_options_t options;
    cpv_options_init(&options);
    options.required_capabilities = required_capabilities;
    options.required_count = ARRAY_SIZE(required_capabilities);
    options.instance_name = "ardupilot-sitl";
    char error[1024];
    const cpv_status_t status = cpv_open_model(model, &options, &vehicle, error, sizeof(error));
    if (status != CPV_OK) {
        AP_HAL::panic("cpvehicle: cannot open %s: %s: %s", colon + 1,
                      cpv_status_string(status), error);
    }

    // Explicit mapping from SITL servo outputs to FMU pwm elements.
    channel_count = cpv_pwm_count(vehicle);
    if (channel_count > max_channels) {
        AP_HAL::panic("cpvehicle: model has %u PWM channels; SITL provides %u",
                      unsigned(channel_count), unsigned(max_channels));
    }
    for (uint8_t i = 0; i < max_channels; i++) {
        channel_source[i] = i < channel_count ? int8_t(i) : -1;
    }
    const char *map = getenv("CPV_CHANNEL_MAP");
    if (map != nullptr && map[0] != '\0') {
        size_t element = 0;
        for (const char *p = map; *p != '\0'; element++) {
            char *end = nullptr;
            const long servo = strtol(p, &end, 10);
            if (end == p || servo < -1 || servo >= max_channels || element >= channel_count ||
                (*end != ',' && *end != '\0')) {
                AP_HAL::panic("cpvehicle: invalid CPV_CHANNEL_MAP '%s' for %u channel(s)", map,
                              unsigned(channel_count));
            }
            channel_source[element] = int8_t(servo);
            p = *end == ',' ? end + 1 : end;
        }
        if (element != channel_count) {
            AP_HAL::panic("cpvehicle: CPV_CHANNEL_MAP has %u entries; the model has %u channels",
                          unsigned(element), unsigned(channel_count));
        }
    }
    ::printf("cpvehicle: %s (%s, linked), %u PWM channel(s), profile %s\n", model->name,
             cpv_model_name(vehicle), unsigned(channel_count), cpv_profile(vehicle));
    for (size_t i = 0; i < channel_count; i++) {
        double start_us = 0;
        cpv_pwm_start(vehicle, i, &start_us);
        ::printf("cpvehicle:   pwm[%u] <- SERVO%d (start %.0f us)\n", unsigned(i + 1),
                 channel_source[i] + 1, start_us);
    }
}

CPVehicle::~CPVehicle()
{
    cpv_close(vehicle);
}

void CPVehicle::fail(const char *what, cpv_status_t status) const
{
    // Never continue on stale physics.
    AP_HAL::panic("cpvehicle: %s at t=%.6f s: %s: %s", what, cpv_time(vehicle),
                  cpv_status_string(status), cpv_last_error(vehicle));
}

/*
  The FMU's geodetic origin is SITL's home, so the vehicle's NED position
  and SITL's location agree. SITL has no geoid: home altitude is used as the
  WGS84 ellipsoid altitude.
 */
void CPVehicle::start()
{
    cpv_status_t status;
    if ((status = cpv_set_parameter(vehicle, "lat0", home.lat * 1.0e-7)) != CPV_OK ||
        (status = cpv_set_parameter(vehicle, "lon0", home.lng * 1.0e-7)) != CPV_OK ||
        (status = cpv_set_parameter(vehicle, "ground_alt_wgs84", home.alt * 0.01)) != CPV_OK) {
        fail("setting the geodetic origin", status);
    }
    // The model's Earth field is SITL's (ArduPilot's World Magnetic Model
    // tables) at home, so its magnetometer agrees with the autopilot's
    // declination. Models without these parameters keep their own field.
    const Vector3f field_ga = AP_Declination::get_earth_field_ga(home);
    const char *const field_names[3] = {"earth_field_north_T", "earth_field_east_T",
                                        "earth_field_down_T"};
    bool field_set = true;
    for (uint8_t i = 0; i < 3; i++) {
        field_set = cpv_set_parameter(vehicle, field_names[i], field_ga[i] * 1.0e-4) == CPV_OK &&
                    field_set;
    }
    status = cpv_initialize(vehicle, 0.0);
    if (status != CPV_OK && status != CPV_WARNING) {
        fail("initialize", status);
    }
    start_time_us = time_now_us;
    started = true;
    cpv_mag_t mag;
    cpv_gnss_t gnss;
    cpv_baro_t baro;
    // A model whose Earth field SITL cannot set has its own, which need not
    // agree with the autopilot's declination: SITL's compass is used then.
    model_mag = field_set && cpv_get_mag(vehicle, &mag) < CPV_UNSUPPORTED;
    model_gnss = cpv_get_gnss(vehicle, &gnss) < CPV_UNSUPPORTED;
    model_baro = cpv_get_baro(vehicle, &baro) < CPV_UNSUPPORTED;
    ::printf("cpvehicle: sensors from the model: IMU%s%s%s; Earth field %s\n",
             model_mag ? ", magnetometer" : "", model_gnss ? ", GNSS" : "",
             model_baro ? ", barometer" : "",
             field_set ? "set from SITL's WMM at home" : "the model's own");
    if (!is_zero(sitl->wind_speed)) {
        ::printf("cpvehicle: SIM_WIND_SPD is ignored; the model provides no wind input\n");
    }
}

void CPVehicle::update(const struct sitl_input &input)
{
    if (!started) {
        start();
    }

    // Actuation: SITL servo outputs to the FMU pwm vector. A zero output
    // means the channel is not driven; the FMU keeps its previous value.
    for (size_t i = 0; i < channel_count; i++) {
        const int8_t servo = channel_source[i];
        if (servo >= 0 && input.servos[servo] != 0) {
            const cpv_status_t status = cpv_set_pwm(vehicle, i, input.servos[servo]);
            if (status != CPV_OK) {
                fail("set pwm", status);
            }
        }
    }

    // Advance to the end of this frame, in integer microseconds from start.
    const double target_s = (time_now_us + frame_time_us - start_time_us) * 1.0e-6;
    double reached_s = 0;
    cpv_status_t status = cpv_advance_to(vehicle, target_s, &reached_s);
    if (status != CPV_OK && status != CPV_WARNING) {
        fail("advance", status);
    }

    cpv_kinematic_state_t state;
    cpv_imu_t imu;
    if ((status = cpv_get_state(vehicle, &state)) >= CPV_UNSUPPORTED) {
        fail("read kinematic state", status);
    }
    if ((status = cpv_get_imu(vehicle, &imu)) >= CPV_UNSUPPORTED) {
        fail("read imu", status);
    }

    // Attitude and position are truth at the vehicle's combined CG. The IMU
    // (gyro and accel_body) is the model's own IMU, so the IMU model selected
    // in the vehicle's configuration is what the autopilot reads; SITL's
    // SIM_GYR*_RND / SIM_ACC*_RND noise (default 0) would add to it.
    Quaternion q(state.quaternion_wb[0], state.quaternion_wb[1], state.quaternion_wb[2],
                 state.quaternion_wb[3]);
    q.normalize();
    q.rotation_matrix(dcm);
    gyro = Vector3f(imu.gyro_frd[0], imu.gyro_frd[1], imu.gyro_frd[2]);
    accel_body = Vector3f(imu.accel_frd[0], imu.accel_frd[1], imu.accel_frd[2]);
    velocity_ef = Vector3f(state.velocity_ned[0], state.velocity_ned[1], state.velocity_ned[2]);
    position = Vector3d(state.position_ned[0], state.position_ned[1], state.position_ned[2]);
    // The FMU's origin is home; SITL's origin moves at 1 Hz (update_position).
    position.xy() += origin.get_distance_NE_double(home);

    // No wind input in the model: air-relative velocity is ground velocity.
    wind_ef.zero();
    velocity_air_ef = velocity_ef;
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
    update_eas_airspeed();

    // No battery capability in the model: a constant SIM_BATT_VOLTAGE.
    battery_voltage = sitl->batt_voltage;
    battery_current = 0;

    update_position();
    time_advance();
    update_mag_field_bf();
    read_model_sensors();
}

/*
  The model's own magnetometer, GNSS and barometer, where it has them, replace
  SITL's simulation of those sensors (SITL's SIM_MAG/SIM_GPS/SIM_BARO error
  settings still apply on top where the drivers add them).
 */
void CPVehicle::read_model_sensors()
{
    cpv_status_t status;
    if (model_mag) {
        cpv_mag_t mag;
        if ((status = cpv_get_mag(vehicle, &mag)) >= CPV_UNSUPPORTED) {
            fail("read magnetometer", status);
        }
        // Gauss to SITL's milligauss, body FRD.
        mag_bf = Vector3f(mag.field_frd_gauss[0], mag.field_frd_gauss[1],
                          mag.field_frd_gauss[2]) * 1000.0f;
    }
    if (model_gnss) {
        cpv_gnss_t gnss;
        if ((status = cpv_get_gnss(vehicle, &gnss)) >= CPV_UNSUPPORTED) {
            fail("read GNSS", status);
        }
        model_sensors.gps_valid = true;
        model_sensors.gps_latitude = gnss.latitude_deg;
        model_sensors.gps_longitude = gnss.longitude_deg;
        model_sensors.gps_altitude = gnss.altitude_wgs84_m;
        model_sensors.gps_velocity_ned = Vector3f(gnss.velocity_ned[0], gnss.velocity_ned[1],
                                                  gnss.velocity_ned[2]);
    }
    if (model_baro) {
        cpv_baro_t baro;
        if ((status = cpv_get_baro(vehicle, &baro)) >= CPV_UNSUPPORTED) {
            fail("read barometer", status);
        }
        model_sensors.baro_valid = true;
        model_sensors.baro_pressure_pa = baro.pressure_pa;
        model_sensors.baro_temperature_c = baro.temperature_c;
    }
}

#endif  // AP_SIM_CPVEHICLE_ENABLED
