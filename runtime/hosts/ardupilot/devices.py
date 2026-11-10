"""ArduPilot's optional hardware in SITL: what the autopilot needs to see each
device, as parameters and SITL serial devices.

Two sources:

- the vehicle's peripherals ([peripherals.KIND] in its configuration;
  Systems.Peripherals): the model computes the readings and SIM_CPVehicle
  hands them to the SITL drivers (hooks.patch); here, the driver parameters;
- the run file's devices ([sitl] devices = ["adsb", "gripper", ...]): devices
  ArduPilot SITL simulates itself (or that need no simulation), switched on.

resolve() returns the parameters, the SITL --serialN arguments and notes.
Serial devices get SERIAL5..SERIAL8 in order; payload servos get outputs 9..16.
Every parameter name was checked against ArduPilot 4.6.2 (libraries/*,
Tools/autotest).
"""
from __future__ import annotations

import math

# Rangefinder orientation (RNGFND1_ORIENT): ArduPilot's Rotation enum.
ORIENT = {"forward": 0, "right": 2, "back": 4, "left": 6, "up": 24, "down": 25}

# Devices SITL simulates (or that need no simulation), by name:
#   title, ArduPilot's page, parameters, SIM_ parameters, serial device
#   (sim: name, SERIALn_PROTOCOL, baud) or None, servo functions (in order),
#   what a waf build needs (None: the default build), and notes.
DEVICES: dict[str, dict] = {
    "adsb": dict(
        title="ADS-B receiver", page="ADS-B Receiver",
        params={"ADSB_TYPE": 1},
        sim={"SIM_ADSB_COUNT": 10, "SIM_ADSB_RADIUS": 10000, "SIM_ADSB_ALT": 1000, "SIM_ADSB_TYPES": 1},
        notes="10 simulated aircraft within 10 km; the simulator takes SERIAL1 (TCP 5762)."),
    "blackbox_logger": dict(
        title="Blackbox logger", page="BlackBox Logger using an AutoPilot",
        params={"LOG_BACKEND_TYPE": 1, "LOG_DISARMED": 1},
        notes="Logs to a file, also while disarmed, as a dedicated logging autopilot would."),
    "button": dict(
        title="Button inputs", page="Button Inputs",
        params={"BTN_ENABLE": 1, "BTN_PIN1": 3, "BTN_FUNC1": 0},
        sim={"SIM_PIN_MASK": 0},
        notes="Press: set bit 3 of SIM_PIN_MASK (param set SIM_PIN_MASK 8)."),
    "buzzer": dict(
        title="Buzzer", page="Buzzer",
        params={"NTF_BUZZ_TYPES": 1, "NTF_BUZZ_PIN": 8},
        notes="Tones are logged; audible only with a --enable-sfml-audio build (SIM_BZ_*)."),
    "camera": dict(
        title="Camera", page="Cameras & Gimbals",
        params={"CAM1_TYPE": 1}, servos=[10],
        notes="Servo-triggered camera: the shutter is a servo output; MAVProxy 'camera trigger'."),
    "gimbal": dict(
        title="Gimbal (servo)", page="Cameras & Gimbals",
        params={"MNT1_TYPE": 1, "MNT1_PITCH_MIN": -90, "MNT1_PITCH_MAX": 25},
        servos=[8, 7, 6],
        notes="Servo gimbal: tilt, roll, pan outputs (mount functions 8, 7, 6); MAVProxy 'gimbal'."),
    "companion_computer": dict(
        title="Companion computer", page="Companion Computers",
        params={"SERIAL2_PROTOCOL": 2},
        notes="MAVLink 2 on SERIAL2: TCP port 5763 + 10 x instance."),
    "sprayer": dict(
        title="Crop sprayer", page="Crop Sprayer",
        params={"SPRAY_ENABLE": 1}, sim={"SIM_SPR_ENABLE": 1}, servos=[22, 23],
        sim_pins={"SIM_SPR_PUMP": 0, "SIM_SPR_SPIN": 1},
        notes="Pump and spinner outputs; MAVProxy 'rc 9 2000' with RC9_OPTION 15, or the sprayer auxiliary function."),
    "display": dict(
        title="Onboard display", page="Display (Onboard)",
        params={"NTF_DISPLAY_TYPE": 10}, build="--enable-sfml --sitl-osd",
        notes="A window on the SITL host; needs a waf build with --enable-sfml --sitl-osd."),
    "dronecan": dict(
        title="DroneCAN peripherals", page="DroneCAN Adapter Node; DroneCAN Peripherals",
        params={"CAN_P1_DRIVER": 1, "CAN_D1_PROTOCOL": 1}, sim={"SIM_CAN_TYPE1": 1},
        build="AP_Periph for sitl_periph_universal, run alongside",
        notes="SITL's virtual CAN bus; peripherals come from a separate AP_Periph SITL process."),
    "esc_telemetry": dict(
        title="ESC telemetry", page="ESCs and Motors",
        sim={"SIM_ESC_TELEM": 1, "SIM_ESC_ARM_RPM": 500},
        notes="ESC rpm, voltage and current telemetry from SITL's motor model."),
    "ethernet": dict(
        title="Ethernet", page="Ethernet Adapters and Switches",
        params={"NET_ENABLE": 1, "NET_P1_TYPE": 1, "NET_P1_PROTOCOL": 2, "NET_P1_PORT": 16001,
                "NET_P1_IP0": 127, "NET_P1_IP1": 0, "NET_P1_IP2": 0, "NET_P1_IP3": 1},
        build="a gcc build (networking is off with clang)",
        notes="MAVLink over UDP 127.0.0.1:16001."),
    "fpv_osd": dict(
        title="FPV on-screen display (MSP)", page="First Person View Video; On-Screen Display (OSD)",
        params={"OSD_TYPE": 3}, serial=("tcp", 33, None),
        notes="MSP DisplayPort OSD on a TCP serial port (a goggle/VTX's view); no video."),
    "osd": dict(
        title="On-screen display (SITL)", page="On-Screen Display (OSD)",
        params={"OSD_TYPE": 2}, sim={"SIM_OSD_COLUMNS": 30, "SIM_OSD_ROWS": 16},
        build="--enable-sfml --sitl-osd",
        notes="An OSD window on the SITL host; needs a waf build with --enable-sfml --sitl-osd."),
    "efi_fuel": dict(
        title="Engine (EFI) fuel flow and level", page="Fuel Flow and Level Sensors",
        params={"EFI_TYPE": 8, "BATT2_MONITOR": 27}, sim={"SIM_EFI_TYPE": 8},
        serial=("hirth", 24, None),
        notes="A Hirth engine's ECU reporting fuel flow and consumption (BATT2 as a fuel monitor)."),
    "generator": dict(
        title="Generator (fuel cell)", page="Generators",
        params={"GEN_TYPE": 2, "BATT2_MONITOR": 18, "BATT3_MONITOR": 17}, sim={"SIM_IE24_ENABLE": 1},
        serial=("ie24", 30, 115200),
        notes="Intelligent Energy 2.4 kW fuel cell: its tank (BATT2) and stack output (BATT3)."),
    "gripper": dict(
        title="Gripper", page="Grippers",
        params={"GRIP_ENABLE": 1, "GRIP_TYPE": 1, "GRIP_GRAB": 2000, "GRIP_RELEASE": 1000},
        sim={"SIM_GRPS_ENABLE": 1}, servos=[28], sim_pins={"SIM_GRPS_PIN": 0},
        notes="Servo gripper; MAVProxy 'gripper grab' / 'gripper release'."),
    "joystick": dict(
        title="Joystick or gamepad", page="Joystick or Gamepad",
        notes="Through the ground station (MAVLink MANUAL_CONTROL / RC override): MAVProxy 'module load joystick'."),
    "landing_gear": dict(
        title="Retractable landing gear", page="Landing Gear/ Retractable Camera Mount",
        params={"LGR_ENABLE": 1}, servos=[29],
        notes="Retract servo; deploys and retracts with altitude (LGR_DEPLOY_ALT, LGR_RETRACT_ALT) or by command."),
    "leds": dict(
        title="LEDs (external)", page="LEDs (external)",
        params={"NTF_LED_TYPES": 123079},
        notes="Notify LEDs; an LED window with a --enable-sfml --sitl-rgbled build."),
    "parachute": dict(
        title="Parachute (release)", page="Parachute",
        params={"CHUTE_ENABLED": 1, "CHUTE_TYPE": 10}, sim={"SIM_PARA_ENABLE": 1}, servos=[27],
        sim_pins={"SIM_PARA_PIN": 0},
        notes="Servo release; MAVProxy 'parachute release' (or crash detection)."),
    "rc_crsf": dict(
        title="RC receiver (CRSF)", page="Radio Control Systems",
        params={"RC_PROTOCOLS": 1}, serial=("crsf", 23, None),
        notes="A CRSF receiver on a serial port; RC still comes from the ground station."),
    "relay": dict(
        title="Relay switch", page="Relay Switch",
        params={"RELAY1_FUNCTION": 1},
        notes="Relay 1 on the SITL default pin (13); MAVProxy 'relay set 0 1'."),
    "remote_id": dict(
        title="Remote ID", page="Remote Id (aka Drone Id)",
        params={"DID_ENABLE": 1, "DID_MAVPORT": 2, "DID_OPTIONS": 0},
        notes="OpenDroneID messages on SERIAL2 (MAVLink); no transmitter is simulated."),
    "safety_switch": dict(
        title="Safety switch", page="Safety Switch",
        params={"BRD_SAFETY_DEFLT": 1},
        notes="Starts with the safety on: MAVProxy 'safety off' before arming."),
    "servo_model": dict(
        title="Servo dynamics (SITL)", page="Servos and Actuators",
        sim={"SIM_SERVO_SPEED": 0.14, "SIM_SERVO_DELAY": 0.0, "SIM_SERVO_FILTER": 0},
        notes="SITL's own servo rate limit, for vehicles without a servo model (S0/S1 model their own)."),
    "smart_battery": dict(
        title="Smart battery (SMBus)", page="Smart Batteries",
        params={"BATT_MONITOR": 16, "BATT_I2C_BUS": 2, "BATT_I2C_ADDR": 11},
        notes="SITL's SMBus battery (cells, temperature, remaining); replaces the analog power module."),
    "telemetry_radio": dict(
        title="Telemetry radio", page="Telemetry Radio",
        params={"SERIAL1_PROTOCOL": 2, "SERIAL1_BAUD": 57}, sim={"SIM_BAUDLIMIT_EN": 1},
        notes="SERIAL1 limited to 57600 baud as a radio link (TCP 5762)."),
    "winch": dict(
        title="Winch", page="Winch",
        params={"WINCH_TYPE": 1}, servos=[88],
        notes="PWM winch; MAVProxy 'winch rate 0.5' / 'winch length 2'."),
    "external_baro": dict(
        title="Second barometer", page="Barometer (external)",
        params={"BARO_EXT_BUS": 2, "BARO_PROBE_EXT": 4}, sim={"SIM_BARO_COUNT": 2},
        notes="A second (I2C) barometer simulated by SITL from the truth."),
    "second_gps": dict(
        title="Second GPS", page="GPS/Compass",
        params={"GPS2_TYPE": 1, "GPS_AUTO_SWITCH": 2}, sim={"SIM_GPS2_DISABLE": 0, "SIM_GPS2_TYPE": 1},
        notes="A second GPS simulated by SITL from the truth, blended with the first."),
}

# ArduPilot pages with no SITL simulation, and why.
UNSUPPORTED = {
    "ais": "AIS: compiled only into Rover (Copter and Plane have stubs).",
    "power_tether": "Power tether: SITL has no tether simulation (a CP Models payload models it).",
    "ppm_encoder": "PPM encoder: SITL takes RC from the ground station; there is no wire-level PPM.",
    "hd_video": "Video (High Definition): nothing in SITL.",
}

# SERIAL0-2 are MAVLink (TCP 5760, 5762, 5763), SERIAL3 and SERIAL4 SITL's GPS 1 and 2.
SERIAL_PORTS = [5, 6, 7, 8]
SERVO_OUTPUTS = list(range(9, 17))


class Plan:
    def __init__(self) -> None:
        self.params: dict[str, float] = {}
        self.serial: list[str] = []
        self.notes: list[str] = []
        self.servos: dict[str, list[int]] = {}
        self._ports = list(SERIAL_PORTS)
        self._outputs = list(SERVO_OUTPUTS)

    def port(self, sim: str, protocol: int, baud: int | None, why: str) -> int:
        if not self._ports:
            raise ValueError(f"{why}: no free SITL serial port (SERIAL5..8 taken)")
        n = self._ports.pop(0)
        self.serial.append(f"--serial{n}={sim}" if sim.startswith("tcp") else f"--serial{n}=sim:{sim}")
        self.params[f"SERIAL{n}_PROTOCOL"] = protocol
        if baud:
            self.params[f"SERIAL{n}_BAUD"] = baud // 1000 if baud >= 1000 else baud
        return n

    def output(self, function: int, why: str) -> int:
        if not self._outputs:
            raise ValueError(f"{why}: no free servo output (9..16 taken)")
        n = self._outputs.pop(0)
        self.params[f"SERVO{n}_FUNCTION"] = function
        return n


def _num(t: dict, key: str, default: float) -> float:
    v = t.get(key, default)
    return float(v)


def peripherals(plan: Plan, physics: dict, home: list[float]) -> None:
    """Driver parameters for the vehicle's [peripherals]."""
    p = physics.get("peripherals", {})
    kind = physics.get("vehicle", "multirotor")
    if "rangefinder" in p:
        r = p["rangefinder"]
        orient = r.get("orientation", "down")
        plan.params.update({
            "RNGFND1_TYPE": 100,
            "RNGFND1_MIN_CM": round(100 * _num(r, "minRange", 0.1)),
            "RNGFND1_MAX_CM": round(100 * _num(r, "maxRange", 12.0)),
            "RNGFND1_ORIENT": ORIENT.get(orient, 25) if isinstance(orient, str) else 25,
        })
    if "airspeed" in p:
        plan.params.update({"ARSPD_TYPE": 100, "SIM_ARSPD_RND": 0})
        if kind == "fixed_wing":
            plan.params["ARSPD_USE"] = 1
    if "optical_flow" in p:
        plan.params.update({"FLOW_TYPE": 10, "SIM_FLOW_ENABLE": 1, "SIM_FLOW_RND": 0})
        if "rangefinder" not in p:
            plan.notes.append("optical_flow: ArduPilot's flow fusion needs a rangefinder; add [peripherals.rangefinder]")
    if "rpm" in p:
        plan.params.update({"RPM1_TYPE": 10, "RPM2_TYPE": 10})
    if "power_module" in p:
        b = p["power_module"].get("battery", {})
        plan.params["BATT_MONITOR"] = 4
        if isinstance(b, dict) and "capacity" in b:
            plan.params["BATT_CAPACITY"] = round(1000 * float(b["capacity"]))
        plan.params["SIM_BATT_CAP_AH"] = 0
    if "temperature" in p:
        plan.params.update({"TEMP1_TYPE": 1, "TEMP1_BUS": 1, "TEMP1_ADDR": 119})
    if "external_ahrs" in p:
        plan.port("VectorNav", 36, 230400, "external_ahrs")
        # A sensor beside the EKF; navigating on it (AHRS_EKF_TYPE 11 with
        # GPS1_TYPE 21) trips a floating-point exception in 4.6.2's arming
        # GPS checks in SITL, so that stays the user's choice.
        plan.params.update({"EAHRS_TYPE": 1, "EAHRS_SENSORS": 0})
        plan.notes.append("external_ahrs: SITL's VectorNav serial simulator reports the model's AHRS; "
                          "the EKF stays the source (to fly on it: AHRS_EKF_TYPE 11, GPS1_TYPE 21, EAHRS_SENSORS 15)")
    if "visual_odometry" in p:
        plan.port("vicon:", 2, None, "visual_odometry")
        # Fused as the second source set; GPS stays the first (switch with an
        # RC option 90 or MAV_CMD_SET_EKF_SOURCE_SET).
        plan.params.update({"VISO_TYPE": 1, "SIM_VICON_TMASK": 3, "EK3_SRC2_POSXY": 6,
                            "EK3_SRC2_VELXY": 6, "EK3_SRC2_POSZ": 1, "EK3_SRC2_VELZ": 6,
                            "EK3_SRC2_YAW": 6})
        plan.notes.append("visual_odometry: EKF source set 2 (GPS stays set 1); "
                          "MAVProxy 'param set EK3_SRC1_POSXY 6' to fly on it")
    if "beacon" in p:
        lat, lon, alt = home[0], home[1], home[2]
        plan.params.update({"BCN_TYPE": 10, "BCN_LATITUDE": lat, "BCN_LONGITUDE": lon,
                            "BCN_ALT": alt, "BCN_ORIENT_YAW": 0, "EK3_SRC3_POSXY": 4,
                            "EK3_SRC3_POSZ": 1, "EK3_SRC3_VELXY": 0})
        plan.notes.append("beacon: EKF source set 3, beacon origin at home")
    if "precision_landing" in p:
        plan.params.update({"PLND_ENABLED": 1, "PLND_TYPE": 4})
        t = p["precision_landing"].get("target", [0.0, 0.0, 0.0])
        lat, lon = _offset(home, float(t[0]), float(t[1]))
        # SITL's own target (unused with the model's camera), kept consistent.
        plan.params.update({"SIM_PLD_LAT": lat, "SIM_PLD_LON": lon})


def _offset(home: list[float], north: float, east: float) -> tuple[float, float]:
    r = 6378137.0
    lat = home[0] + math.degrees(north / r)
    lon = home[1] + math.degrees(east / (r * math.cos(math.radians(home[0]))))
    return round(lat, 9), round(lon, 9)


def devices(plan: Plan, names: list[str]) -> None:
    """Parameters and serial devices for the run file's [sitl] devices."""
    for name in names:
        if name in UNSUPPORTED:
            raise ValueError(f"device {name!r}: {UNSUPPORTED[name]}")
        d = DEVICES.get(name)
        if d is None:
            raise ValueError(f"device {name!r}: one of {', '.join(sorted(DEVICES))}")
        plan.params.update(d.get("params", {}))
        plan.params.update(d.get("sim", {}))
        if d.get("serial"):
            sim, protocol, baud = d["serial"]
            plan.port(sim if sim != "tcp" else f"tcp:{len(plan.serial) + 5}", protocol, baud, name)
        outputs = [plan.output(f, name) for f in d.get("servos", [])]
        if outputs:
            plan.servos[name] = outputs
        for pin, index in d.get("sim_pins", {}).items():
            plan.params[pin] = outputs[index]
        if d.get("build"):
            plan.notes.append(f"{name}: needs {d['build']}")
        if name == "adsb" and "telemetry_radio" in names:
            raise ValueError("adsb and telemetry_radio both take SERIAL1")


def resolve(physics: dict, names: list[str], home: list[float]) -> Plan:
    plan = Plan()
    peripherals(plan, physics, home)
    devices(plan, names)
    return plan


def catalog() -> dict:
    """The devices a run can switch on, for frontends."""
    return {
        "devices": {k: {"title": d["title"], "page": d["page"], "notes": d["notes"],
                        "build": d.get("build")} for k, d in DEVICES.items()},
        "unsupported": UNSUPPORTED,
    }
