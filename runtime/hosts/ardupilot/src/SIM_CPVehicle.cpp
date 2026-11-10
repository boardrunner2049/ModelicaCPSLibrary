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
    has_rangefinder = cpv_has_capability(vehicle, "rangefinder_v1");
    has_airspeed = cpv_has_capability(vehicle, "airspeed_v1");
    has_flow = cpv_has_capability(vehicle, "optical_flow_v1");
    has_rpm = cpv_has_capability(vehicle, "rpm_v1");
    has_battery = cpv_has_capability(vehicle, "battery_v1");
    has_temperature = cpv_has_capability(vehicle, "temperature_v1");
    has_ahrs = cpv_has_capability(vehicle, "external_ahrs_v1");
    has_odometry = cpv_has_capability(vehicle, "visual_odometry_v1");
    has_beacon = cpv_has_capability(vehicle, "beacon_v1");
    has_precland = cpv_has_capability(vehicle, "precision_landing_v1");
    aux_count = cpv_aux_count(vehicle);
    if (aux_count > 0) {
        ::printf("cpvehicle: payload servos: aux_pwm[1..%u] <- SERVO9..%u\n", unsigned(aux_count),
                 unsigned(8 + aux_count));
    }
    if (has_rpm) {
        double rpm_values[max_channels];
        for (size_t n = 1; n <= max_channels && rpm_count == 0; n++) {
            if (cpv_get_field(vehicle, "rpm", rpm_values, n) == CPV_OK) {
                rpm_count = n;
            }
        }
        // AP_RPM_SITL reads the shafts of the motor mask
        motor_mask = rpm_count >= 32 ? 0xFFFFFFFFU : (1U << rpm_count) - 1;
    }
    ::printf("cpvehicle: peripherals from the model:%s%s%s%s%s%s%s%s%s%s\n",
             has_rangefinder ? " rangefinder" : "", has_airspeed ? " airspeed" : "",
             has_flow ? " optical-flow" : "", has_rpm ? " rpm" : "",
             has_battery ? " battery" : "", has_temperature ? " temperature" : "",
             has_ahrs ? " external-AHRS" : "", has_odometry ? " visual-odometry" : "",
             has_beacon ? " beacon" : "", has_precland ? " precision-landing" : "");
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

    // Payload servos: SERVO9.. drive aux_pwm, held while not driven.
    for (size_t k = 0; k < aux_count && 8 + k < max_channels; k++) {
        if (input.servos[8 + k] != 0) {
            const cpv_status_t status = cpv_set_aux_pwm(vehicle, k, input.servos[8 + k]);
            if (status != CPV_OK) {
                fail("set aux pwm", status);
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
    read_peripherals();
}

void CPVehicle::field(const char *name, double *values, size_t count) const
{
    const cpv_status_t status = cpv_get_field(vehicle, name, values, count);
    if (status >= CPV_UNSUPPORTED) {
        fail(name, status);
    }
}

/*
  The model's peripherals replace SITL's simulation of those devices: the
  rangefinder, RPM and battery through the Aircraft's own outputs, the others
  through model_sensors, which the SITL drivers read (hooks.patch).
 */
void CPVehicle::read_peripherals()
{
    double v[16];
    auto &m = model_sensors;
    // NED from the model's origin (home) to a location
    auto located = [this](const double *ned) {
        Location loc = home;
        loc.offset(ned[0], ned[1]);
        loc.alt = home.alt - int32_t(ned[2] * 100.0);
        return loc;
    };
    if (has_rangefinder) {
        // The device's report: out of range reads as such, no return as no data.
        field("rangefinder_distance_m", v, 1);
        rangefinder_m[0] = v[0] >= 0 ? float(v[0]) : nanf("");
    }
    if (has_rpm) {
        field("rpm", v, MIN(rpm_count, ARRAY_SIZE(v)));
        for (size_t i = 0; i < rpm_count && i < ARRAY_SIZE(v); i++) {
            rpm[i] = v[i];
        }
    }
    if (has_battery) {
        field("battery_voltage_v", v, 1);
        battery_voltage = v[0];
        field("battery_current_a", v, 1);
        battery_current = v[0];
    }
    if (has_airspeed) {
        field("airspeed_differential_pressure_pa", v, 1);
        m.airspeed_present = true;
        m.airspeed_pressure_pa = v[0];
    }
    if (has_flow) {
        double ok;
        field("flow_valid", &ok, 1);
        field("flow_rate_xy", v, 2);
        field("flow_body_rate_xy", v + 2, 2);
        m.flow_present = true;
        m.flow_valid = ok > 0.5;
        m.flow_rate = Vector2f(v[0], v[1]);
        m.flow_body_rate = Vector2f(v[2], v[3]);
    }
    if (has_temperature) {
        field("temperature_k", v, 1);
        m.temperature_present = true;
        m.temperature_c = v[0] - 273.15;
    }
    if (has_odometry) {
        double ok;
        field("odometry_valid", &ok, 1);
        field("odometry_position_ned", v, 3);
        field("odometry_velocity_ned", v + 3, 3);
        field("odometry_quaternion", v + 6, 4);
        m.vision_present = true;
        m.vision_valid = ok > 0.5;
        m.vision_location = located(v);
        m.vision_position_ned = Vector3d(v[0], v[1], v[2]);
        m.vision_velocity_ned = Vector3f(v[3], v[4], v[5]);
        m.vision_attitude = Quaternion(v[6], v[7], v[8], v[9]);
    }
    if (has_beacon) {
        double ok;
        field("beacon_valid", &ok, 1);
        field("beacon_position_ned", v, 3);
        m.beacon_present = true;
        m.beacon_valid = ok > 0.5;
        m.beacon_location = located(v);
    }
    if (has_precland) {
        double ok;
        field("landing_target_valid", &ok, 1);
        field("landing_target_angle", v, 2);
        field("landing_target_distance_m", v + 2, 1);
        m.precland_present = true;
        m.precland_valid = ok > 0.5;
        // A downward camera (PLND_ORIENT 25): its axes are the body's.
        Vector3f los(tanf(v[0]), tanf(v[1]), 1.0f);
        m.precland_los_body = los.normalized();
        m.precland_distance_m = v[2];
    }
    if (has_ahrs) {
        double ok;
        field("ahrs_valid", &ok, 1);
        field("ahrs_quaternion", v, 4);
        field("ahrs_rate_frd", v + 4, 3);
        field("ahrs_accel_frd", v + 7, 3);
        field("ahrs_position_ned", v + 10, 3);
        double vel[3];
        field("ahrs_velocity_ned", vel, 3);
        m.ahrs_present = true;
        m.ahrs_valid = ok > 0.5;
        auto &a = m.ahrs;
        const Location loc = located(v + 10);
        a.latitude = loc.lat * 1.0e-7;
        a.longitude = loc.lng * 1.0e-7;
        a.altitude = home.alt * 0.01 - v[12];
        a.speedN = vel[0];
        a.speedE = vel[1];
        a.speedD = vel[2];
        a.quaternion = Quaternion(v[0], v[1], v[2], v[3]);
        float roll, pitch, yaw;
        a.quaternion.to_euler(roll, pitch, yaw);
        a.rollDeg = degrees(roll);
        a.pitchDeg = degrees(pitch);
        a.yawDeg = wrap_360(degrees(yaw));
        a.rollRate = degrees(v[4]);
        a.pitchRate = degrees(v[5]);
        a.yawRate = degrees(v[6]);
        a.xAccel = v[7];
        a.yAccel = v[8];
        a.zAccel = v[9];
    }
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
