use clap::{Parser, Subcommand, ValueEnum};
use fire_compose::{artifact, catalog, config, parts, Result};
use std::{fs, path::PathBuf, process::Command};

#[derive(Parser)]
#[command(
    version,
    about = "Validate FIRE vehicle TOML and compose standard Modelica source"
)]
struct Cli {
    #[command(subcommand)]
    command: Action,
}
#[derive(Subcommand)]
enum Action {
    /// Validate configuration without writing files or invoking a compiler.
    Validate {
        config: PathBuf,
        /// Read the physics configuration from this TOML table path.
        #[arg(long)]
        config_table: Option<String>,
        #[arg(long)]
        json: bool,
    },
    /// Emit generated Modelica and a provenance manifest.
    Emit(Generate),
    /// Print what a configuration can select (vehicle kinds, parameters, part models) as JSON.
    Catalog,
    /// Show what a real part (parts/KIND/*.toml) becomes at a level: the
    /// parameters, the figures each came from, and what the level cannot represent.
    Part {
        /// imu, gnss, barometer or magnetometer
        kind: String,
        /// A part id, name or path; omit to list the parts of KIND.
        part: Option<String>,
        #[arg(long)]
        level: Option<String>,
        #[arg(long)]
        mode: Option<String>,
        /// none or draw
        #[arg(long)]
        tolerances: Option<String>,
        /// Explicit [KIND] values, as TOML (e.g. 'samplePeriod = 0.1')
        #[arg(long)]
        set: Option<String>,
        /// The report as JSON
        #[arg(long)]
        json: bool,
    },
    /// Emit and run OpenModelica checkModel; this does not simulate an FMU.
    Check {
        #[command(flatten)]
        generate: Generate,
        #[arg(long, default_value = "omc")]
        compiler: PathBuf,
    },
}
#[derive(clap::Args)]
struct Generate {
    config: PathBuf,
    /// Read the physics configuration from this TOML table path.
    #[arg(long)]
    config_table: Option<String>,
    #[arg(long, default_value = "plant")]
    profile: Profile,
    #[arg(long)]
    output_dir: Option<PathBuf>,
    #[arg(long)]
    library_root: Option<PathBuf>,
}
#[derive(Clone, Copy, ValueEnum)]
enum Profile {
    Plant,
    #[value(alias = "fastdyn")]
    Vio,
}
impl Profile {
    fn name(self) -> &'static str {
        match self {
            Self::Plant => "plant",
            Self::Vio => "vio",
        }
    }
}

fn generate(args: &Generate) -> Result<(PathBuf, PathBuf, config::ResolvedFireConfig, PathBuf)> {
    let raw = fs::read_to_string(&args.config).map_err(|e| e.to_string())?;
    let config = config::parse_at(&raw, args.config_table.as_deref())?;
    let root = args
        .library_root
        .clone()
        .unwrap_or_else(artifact::library_root)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    let output = args
        .output_dir
        .clone()
        .unwrap_or_else(|| root.join("build/composed").join(&config.model_name));
    let (model, manifest) = artifact::emit(&config, &raw, args.profile.name(), &root, &output)?;
    let staged = artifact::stage_library(&root, model.parent().unwrap())?;
    Ok((model, manifest, config, staged))
}

fn run() -> Result<()> {
    match Cli::parse().command {
        Action::Validate {
            config: path,
            config_table,
            json,
        } => {
            let raw = fs::read_to_string(path).map_err(|e| e.to_string())?;
            let c = config::parse_at(&raw, config_table.as_deref())?;
            if json {
                println!(
                    "{}",
                    serde_json::to_string_pretty(&c).map_err(|e| e.to_string())?
                );
            } else {
                println!("Configuration valid: {}", c.model_name);
            }
        }
        Action::Catalog => {
            println!(
                "{}",
                serde_json::to_string_pretty(&catalog::catalog()).unwrap()
            );
        }
        Action::Part {
            kind,
            part,
            level,
            mode,
            tolerances,
            set,
            json,
        } => {
            let Some(part) = part else {
                println!(
                    "{}",
                    serde_json::to_string_pretty(&parts::catalog()[&kind]).unwrap()
                );
                return Ok(());
            };
            let mut sel = toml::map::Map::new();
            sel.insert("part".into(), toml::Value::String(part));
            for (k, v) in [("level", level), ("mode", mode), ("tolerances", tolerances)] {
                if let Some(v) = v {
                    sel.insert(k.into(), toml::Value::String(v));
                }
            }
            let explicit: toml::map::Map<String, toml::Value> =
                toml::from_str(set.as_deref().unwrap_or("")).map_err(|e| e.to_string())?;
            let (_, _, report) = parts::resolve(&kind, &sel, &explicit)?;
            if json {
                println!("{}", serde_json::to_string_pretty(&report).unwrap());
            } else {
                print!("{}", parts::summary(&report));
            }
        }
        Action::Emit(args) => {
            let (model, manifest, _, _) = generate(&args)?;
            println!("{}\n{}", model.display(), manifest.display());
        }
        Action::Check {
            generate: args,
            compiler,
        } => {
            let (model, manifest, c, root) = generate(&args)?;
            let work = tempfile::tempdir().map_err(|e| e.to_string())?;
            let quote = |p: PathBuf| serde_json::to_string(&p.to_string_lossy()).unwrap();
            let script=format!("loadModel(Modelica, {{\"4.0.0\"}});\nloadFile({});\nloadFile({});\ncheckModel({});\nif translateModel({}) then print(\"FIRE_TRANSLATED\\n\"); end if;\ngetErrorString();\n",quote(root.join("package.mo")),quote(model.clone()),c.model_name,c.model_name);
            let script_path = work.path().join("check.mos");
            fs::write(&script_path, script).map_err(|e| e.to_string())?;
            let run = Command::new(compiler)
                .arg(script_path)
                .current_dir(work.path())
                .output()
                .map_err(|e| e.to_string())?;
            let log = format!(
                "{}{}",
                String::from_utf8_lossy(&run.stdout),
                String::from_utf8_lossy(&run.stderr)
            );
            let passed = run.status.success()
                && !log.contains("Error:")
                && log.contains("FIRE_TRANSLATED")
                && log.contains(&format!(
                    "Check of {} completed successfully.",
                    c.model_name
                ));
            let mut m: serde_json::Value =
                serde_json::from_slice(&fs::read(&manifest).map_err(|e| e.to_string())?)
                    .map_err(|e| e.to_string())?;
            m["verification"]["modelica_check"] = serde_json::json!(if passed {
                "openmodelica_check_passed"
            } else {
                "failed"
            });
            fs::write(&manifest, serde_json::to_string_pretty(&m).unwrap() + "\n")
                .map_err(|e| e.to_string())?;
            print!("{log}");
            if !passed {
                return Err("OpenModelica checkModel failed".into());
            }
        }
    }
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("fire-compose: {e}");
        std::process::exit(2);
    }
}
