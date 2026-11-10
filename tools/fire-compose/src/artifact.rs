//! Deterministic artifacts and provenance, separate from compilation status.
use crate::{
    config::ResolvedFireConfig, lower, Result, GENERATOR, HEADER, LEGACY_GENERATOR, LEGACY_HEADER,
    PREVIOUS_GENERATOR, PREVIOUS_HEADER,
};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Component, Path, PathBuf},
    process::Command,
};

pub fn hash(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
pub fn library_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("FIRE library root")
}

fn resolved(path: &Path) -> Result<PathBuf> {
    let absolute = if path.is_absolute() {
        path.to_owned()
    } else {
        std::env::current_dir()
            .map_err(|e| e.to_string())?
            .join(path)
    };
    let mut result = PathBuf::new();
    for c in absolute.components() {
        match c {
            Component::ParentDir => {
                result.pop();
            }
            Component::CurDir => {}
            _ => {
                result.push(c);
                if fs::symlink_metadata(&result).is_ok() {
                    result = result.canonicalize().map_err(|e| e.to_string())?;
                }
            }
        }
    }
    Ok(result)
}
fn source_files(dir: &Path, root: &Path, files: &mut Vec<PathBuf>) -> Result<()> {
    for item in fs::read_dir(dir).map_err(|e| e.to_string())? {
        let path = item.map_err(|e| e.to_string())?.path();
        if path.is_symlink() {
            continue;
        }
        if path.is_dir() {
            if ![
                "build",
                "target",
                ".git",
                "__pycache__",
                "tools",
                "deprecated",
            ]
            .contains(&path.file_name().unwrap().to_string_lossy().as_ref())
            {
                source_files(&path, root, files)?;
            }
        } else if path.extension().is_some_and(|v| v == "mo")
            || path.file_name().is_some_and(|v| v == "package.order")
        {
            files.push(path.strip_prefix(root).unwrap().to_owned());
        }
    }
    Ok(())
}
pub fn source_identity(root: &Path) -> Result<Value> {
    let mut paths = Vec::new();
    source_files(root, root, &mut paths)?;
    paths.sort();
    let mut digest = Sha256::new();
    let mut files = serde_json::Map::new();
    for p in paths {
        let bytes = fs::read(root.join(&p)).map_err(|e| e.to_string())?;
        let name = p.to_string_lossy().replace('\\', "/");
        digest.update(name.as_bytes());
        digest.update([0]);
        digest.update(&bytes);
        digest.update([0]);
        files.insert(name, Value::String(hash(&bytes)));
    }
    let revision = Command::new("git")
        .args(["-C"])
        .arg(root)
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|r| r.status.success())
        .map(|r| String::from_utf8_lossy(&r.stdout).trim().to_owned());
    Ok(json!({"sha256":format!("{:x}",digest.finalize()),"files":files,"git_revision":revision}))
}

pub fn emit(
    config: &ResolvedFireConfig,
    raw: &str,
    profile: &str,
    root: &Path,
    output: &Path,
) -> Result<(PathBuf, PathBuf)> {
    let profile = crate::canonical_profile(profile)?;
    let mut source = format!("{HEADER}{}", lower::compose(config, profile)?.emit()?);
    // Where every fitted device's parameter values came from (crate::provenance).
    let provenance = crate::provenance::table(config);
    if !provenance.is_empty() {
        let end = format!("end {};", config.model_name);
        if let Some(at) = source.rfind(&end) {
            let info = crate::provenance::html(&provenance).replace('\\', "\\\\").replace('"', "\\\"");
            source.insert_str(at, &format!("  annotation(Documentation(info=\"{info}\"));\n"));
        }
    }
    let root = root.canonicalize().map_err(|e| e.to_string())?;
    let output = resolved(output)?;
    if output.starts_with(&root) && !output.starts_with(root.join("build")) {
        return Err("Output directory must be inside ROOT/build or outside the source root".into());
    }
    let model = output.join(format!("{}.mo", config.model_name));
    let manifest = output.join(format!("{}.manifest.json", config.model_name));
    for p in [&model, &manifest] {
        if p.is_symlink() {
            return Err(format!("Refusing symlink output file: {}", p.display()));
        }
    }
    if model.exists() {
        let previous_source = fs::read_to_string(&model).map_err(|e| e.to_string())?;
        if ![HEADER, PREVIOUS_HEADER, LEGACY_HEADER]
            .iter()
            .any(|header| previous_source.starts_with(header))
        {
            return Err(format!(
                "Refusing to overwrite non-generated Modelica file: {}",
                model.display()
            ));
        }
    }
    let mut previous = None;
    if manifest.exists() {
        let prior: Value = serde_json::from_slice(&fs::read(&manifest).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        if ![GENERATOR, PREVIOUS_GENERATOR, LEGACY_GENERATOR]
            .iter()
            .any(|generator| prior["generator_id"] == *generator)
        {
            return Err("Refusing to overwrite non-generated manifest".into());
        }
        previous = Some(prior);
    }
    let mut manifest_value = if config.vehicle == "multirotor" {
        multirotor_manifest(config, raw, profile, &root, &source)?
    } else {
        vehicle_manifest(config, raw, profile, &root, &source)?
    };
    if !provenance.is_empty() {
        manifest_value["provenance"] = Value::Array(provenance);
    }
    if let Some(prior) = previous {
        if ["config_sha256", "generated_model_sha256", "profile"]
            .iter()
            .all(|key| prior[*key] == manifest_value[*key])
            && prior["source"]["sha256"] == manifest_value["source"]["sha256"]
        {
            if prior["verification"].is_object() {
                manifest_value["verification"] = prior["verification"].clone();
            }
            if prior["compiler"].is_object() {
                manifest_value["compiler"] = prior["compiler"].clone();
            }
        }
    }
    fs::create_dir_all(&output).map_err(|e| e.to_string())?;
    write_if_changed(&model, source.as_bytes())?;
    let text = serde_json::to_string_pretty(&manifest_value).map_err(|e| e.to_string())? + "\n";
    write_if_changed(&manifest, text.as_bytes())?;
    Ok((model, manifest))
}

fn multirotor_manifest(
    config: &ResolvedFireConfig,
    raw: &str,
    profile: &str,
    root: &Path,
    source: &str,
) -> Result<Value> {
    let ground_contact = config.landing_gear["enabled"].as_bool().unwrap();
    let mut manifest_value = json!({
        "generator_id":GENERATOR,"generator_version":env!("CARGO_PKG_VERSION"),"manifest_version":2,
        "core_contract":"ned_frd_continuous_v1",
        "frames":{"world":"NED","body":"FRD","chassis":"FRD","rotor":"local +z thrust","sensor":"local; v_b=R_bs*v_s"},
        "physics":{"propulsion":config.propulsion_level(),"rotor_aero":"R0","body":"rigid_6dof","drag":"none",
                   "contact":if ground_contact { "four_point_spring_damper" } else { "none" },
                   "sensors":"ideal_with_fixed_bias","acquisition":"continuous"},
        "schema_version":config.schema_version,"profile":profile,"model":config.model_name,
        "config_sha256":hash(raw.as_bytes()),"config":config,"source":source_identity(root)?,
        "generated_model_sha256":hash(source.as_bytes()),
        "interface":{"nRotors":config.geometry["nRotors"],"nActuators":config.geometry["nActuators"],
                     "actuatorIndex":config.geometry["actuatorIndex"]},
        "event_requirements":{"sampled_sensors":config.acquisition=="sampled","sampled_actuators":profile=="vio" && config.acquisition=="sampled","ground_contact":ground_contact},
        "verification":{"configuration":"passed","modelica_check":"not_run","fmi_generation":"not_run","fmu_simulation":"not_run","firmware":"not_run"}
    });
    manifest_value["imu"] = json!({"level":config.imu_level(),"class":config.imu_class(),
        "parameters":config.imu,"other_sensors":"continuous_ideal",
        "readout":if config.imu_model()=="ideal" {"continuous"} else {"sampled"},
        "calibration":"user_supplied_not_hardware_validated"});
    manifest_value["imu"]["fidelity_definition"] = json!("digital_errors_electromechanical_v2");
    manifest_value["imu"]["legacy"] = json!(matches!(
        config.imu_model(),
        "statistical" | "device" | "acoustic"
    ));
    if !config.acoustic.is_empty() {
        manifest_value["acoustic_vulnerability"] = json!({"configuration":config.acoustic,
            "pressure_contract":if config.acoustic["model"].as_str()==Some("modal_force") {"instantaneous_Pa"} else {"slow_RMS_envelope_Pa"},
            "package_model":"static_modal_force_gain","calibration":"illustrative_or_user_identified"});
    }
    if config.imu_model() != "ideal" {
        manifest_value["core_contract"] = json!("ned_frd_sampled_imu_v1");
        manifest_value["physics"]["sensors"] = json!("selected_imu_other_sensors_ideal");
        manifest_value["physics"]["acquisition"] = json!("sampled_imu_continuous_others");
        manifest_value["event_requirements"]["sampled_sensors"] = json!(true);
        manifest_value["imu"]["metadata_outputs"] = json!([
            "imuSampleTime",
            "imuDeliveryTime",
            "imuSequence",
            "imuValid"
        ]);
        manifest_value["imu"]["samplePeriod_s"] = json!(config.imu["samplePeriod"]);
    }
    if profile == "vio" {
        manifest_value["input_adapter"] = json!({
            "class":"ModelicaCPSLibrary.Adapters.VirtualIO.PwmInput",
            "input_contract":"pwm_pulse_command_v1",
            "output_contract":"normalized_pwm_command_v1",
            "input_unit":"us", "output_unit":"1", "acquisition":"continuous",
            "normalization":"clamp((pulseWidth_us-pwmMin)/(pwmMax-pwmMin),0,1)",
            "pwm_min_us":config.interface["pwm_min"], "pwm_max_us":config.interface["pwm_max"],
            "propulsion_input":"escs.escCommand",
            "mapping_owner":"composer.geometry.actuatorIndex",
            "command_meaning":if config.three_phase() { "high_side_pwm_duty" } else if config.voltage_driven() { "linear_voltage_modulation" } else { "linear_target_speed_ratio" }
        });
    }
    if config.electromechanical() && !config.three_phase() {
        manifest_value["event_requirements"]["motor_current_zero_crossing"] = json!(true);
    }
    if config.three_phase() {
        manifest_value["event_requirements"]["six_step_commutation"] = json!(true);
        manifest_value["event_requirements"]["diode_conduction"] = json!(true);
        manifest_value["event_requirements"]["coast_release"] = json!(true);
        manifest_value["event_requirements"]["pwm_switching"] = json!(config.switching());
    }
    manifest_value["propulsion_contract"] = if config.three_phase() {
        json!({"command":"esc_command_v1", "control_mode":"forward_six_step_voltage_modulation",
            "mapping":"clamp(normalizedCommand,0,1)", "electrical":"three_phase_abc_floating_star_v1",
            "supply":"shared_resistive_source_and_capacitive_dc_link", "source_can_absorb_energy":true,
            "dcVoltage_V":config.power["dcVoltage"], "power_accounting":true,
            "load_feedback":true,"speed_states_per_rotor":1,"current_states_per_rotor":2,
            "shaft_angle":"theta_e=polePairs*theta_m+theta0", "rotor_position":"ideal_known",
            "drive":if config.switching() {"six_switch_asynchronous_high_side_pwm"} else {"ccm_mean_asynchronous_high_side_pwm"},
            "cutoff":"latched_sink_recirculation_then_hi_z", "regeneration":"body_diode_paths_remain_connected",
            "spin_reaction":"axial_fixed_carrier", "carrier_rotation_feedback":false,
            "gyroscopic_coupling":false,"current_limiting":false,"thermal_dynamics":false,
            "pwm_ripple":config.switching(),"pwm_cycle_discontinuous_conduction":config.switching(),
            "switch_transition_energy":false, "parameter_basis":"per-phase R and effective L; ke is phase peak V per mechanical rad/s",
            "average_validity":"P3 is a CCM approximation; compare matched PWM-period means away from commutation"})
    } else if config.electromechanical() {
        json!({"command":"esc_command_v1", "control_mode":"unipolar_voltage_modulation",
            "mapping":"clamp(normalizedCommand,0,1)", "electrical":"conserving_pins_with_open_circuit_voltage_v1",
            "supply":if profile == "vio" { "internal_constant" } else { "input_default" },
            "dcVoltage_V":config.power["dcVoltage"], "power_accounting":true,
            "load_feedback":true, "speed_states_per_rotor":1, "current_states_per_rotor":1,
            "shaft_angle":"MSL kinematic angle; no angle-dependent physics",
            "drive":"nonregenerative_average", "cutoff":"positive_current_decay_then_open_circuit_coast",
            "spin_reaction":"axial_fixed_carrier", "carrier_rotation_feedback":false,
            "gyroscopic_coupling":false, "current_limiting":false, "thermal_dynamics":false,
            "pwm_ripple":false, "pwm_cycle_discontinuous_conduction":false,
            "parameter_basis":"DC-equivalent SI; k is both torque and back-EMF constant"})
    } else if config.voltage_driven() {
        json!({"command":"esc_command_v1", "control_mode":"unipolar_voltage_modulation",
               "mapping":"clamp(normalizedCommand,0,1)", "voltage":"equivalent_voltage_command_v1",
               "supply":if profile == "vio" { "internal_constant" } else { "input_default" }, "dcVoltage_V":config.power["dcVoltage"],
               "speed_gain_unit":"(rad/s)/V", "power_accounting":false,
               "load_feedback":false, "speed_states_per_rotor":1})
    } else {
        json!({"command":"speed_command_v1", "power_accounting":false,
               "load_feedback":false, "speed_states_per_rotor":1})
    };
    manifest_value["propulsion_components"] = json!({
        "esc": config.esc_class(),
        "motor": config.motor_class(),
        "rotor": "ModelicaCPSLibrary.Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade",
        "vehicle": "ModelicaCPSLibrary.Vehicles.Copter.MultirotorWithSensors",
        "vehicle_input": "rotor-local hub force and moment"
    });
    Ok(manifest_value)
}

/// Rover/fixed-wing: the configuration, provenance and the selected sensors;
/// the export class documents the plant, propulsion and PWM channel meaning.
fn vehicle_manifest(
    config: &ResolvedFireConfig,
    raw: &str,
    profile: &str,
    root: &Path,
    source: &str,
) -> Result<Value> {
    let kind = crate::vehicle::kind(&config.vehicle)?;
    let mut manifest = json!({
        "generator_id":GENERATOR,"generator_version":env!("CARGO_PKG_VERSION"),"manifest_version":2,
        "core_contract":"ned_frd_continuous_v1",
        "frames":{"world":"NED","body":"FRD","sensor":"local; v_b=R_bs*v_s"},
        "vehicle":kind.name,"export":kind.export,"plant":kind.plant,
        "schema_version":config.schema_version,"profile":profile,"model":config.model_name,
        "config_sha256":hash(raw.as_bytes()),"config":config,"source":source_identity(root)?,
        "generated_model_sha256":hash(source.as_bytes()),
        "event_requirements":{"sampled_sensors":config.imu_model()!="ideal"},
        "verification":{"configuration":"passed","modelica_check":"not_run","fmi_generation":"not_run","fmu_simulation":"not_run","firmware":"not_run"}
    });
    manifest["imu"] = json!({"level":config.imu_level(),"class":config.imu_class(),
        "parameters":config.imu,"other_sensors":"continuous_ideal",
        "readout":if config.imu_model()=="ideal" {"continuous"} else {"sampled"},
        "fidelity_definition":"digital_errors_electromechanical_v2",
        "calibration":"user_supplied_not_hardware_validated"});
    if config.imu_model() != "ideal" {
        manifest["core_contract"] = json!("ned_frd_sampled_imu_v1");
        manifest["imu"]["samplePeriod_s"] = json!(config.imu["samplePeriod"]);
    }
    if !config.acoustic.is_empty() {
        manifest["acoustic_vulnerability"] = json!({"configuration":config.acoustic});
    }
    Ok(manifest)
}

fn write_if_changed(path: &Path, bytes: &[u8]) -> Result<()> {
    if fs::read(path).ok().as_deref() != Some(bytes) {
        fs::write(path, bytes).map_err(|e| e.to_string())?;
    }
    Ok(())
}

/// Stage the declared package tree under its Modelica name. Repository tooling
/// and undeclared compatibility directories are not Modelica source roots.
pub fn stage_library(root: &Path, output: &Path) -> Result<PathBuf> {
    let sources = output.join("sources");
    let staged = sources.join("ModelicaCPSLibrary");
    if sources.is_symlink() || staged.is_symlink() {
        return Err("Refusing symlink staged source directories".into());
    }
    let marker = staged.join(".fire-compose");
    if staged.exists() && fs::read_to_string(&marker).ok().as_deref() != Some(GENERATOR) {
        return Err("Refusing to overwrite an unowned staged library".into());
    }
    fn copy_package(src: &Path, dst: &Path) -> Result<()> {
        fs::create_dir_all(dst).map_err(|e| e.to_string())?;
        for name in ["package.mo", "package.order"] {
            if src.join(name).exists() {
                write_if_changed(
                    &dst.join(name),
                    &fs::read(src.join(name)).map_err(|e| e.to_string())?,
                )?;
            }
        }
        let order = fs::read_to_string(src.join("package.order")).map_err(|e| e.to_string())?;
        for name in order.lines().map(str::trim).filter(|s| !s.is_empty()) {
            if !crate::identifier(name) {
                return Err(format!("invalid package.order entry: {name}"));
            }
            if src.join(name).join("package.mo").exists() {
                copy_package(&src.join(name), &dst.join(name))?;
            } else if src.join(format!("{name}.mo")).exists() {
                let file = format!("{name}.mo");
                write_if_changed(
                    &dst.join(&file),
                    &fs::read(src.join(file)).map_err(|e| e.to_string())?,
                )?;
            }
        }
        Ok(())
    }
    fs::create_dir_all(&sources).map_err(|e| e.to_string())?;
    let work = tempfile::tempdir_in(&sources).map_err(|e| e.to_string())?;
    let fresh = work.path().join("ModelicaCPSLibrary");
    copy_package(root, &fresh)?;
    fs::write(fresh.join(".fire-compose"), GENERATOR).map_err(|e| e.to_string())?;
    // Replace only a directory owned by this generator, so removed classes do
    // not survive a subsequent composition and nested symlinks are not followed.
    let old = work.path().join("previous");
    if staged.exists() {
        fs::rename(&staged, &old).map_err(|e| e.to_string())?;
    }
    if let Err(error) = fs::rename(&fresh, &staged) {
        if old.exists() {
            let _ = fs::rename(&old, &staged);
        }
        return Err(error.to_string());
    }
    // Retire previous namespaces only when their trees belong to this generator.
    for (namespace, generator) in [
        ("fire_modelica_models", PREVIOUS_GENERATOR),
        ("FIRE_Modelica", LEGACY_GENERATOR),
    ] {
        let legacy = sources.join(namespace);
        if !legacy.is_symlink()
            && fs::read_to_string(legacy.join(".fire-compose"))
                .ok()
                .as_deref()
                == Some(generator)
        {
            fs::rename(&legacy, work.path().join(namespace)).map_err(|e| e.to_string())?;
        }
    }
    Ok(staged)
}
