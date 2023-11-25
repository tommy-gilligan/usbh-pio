use defmt_json_schema::{v1::JsonFrame, SchemaVersion};
use std::{
    env::args,
    fs::{self, File},
    path::{Path, PathBuf},
    process::Command,
};

const CASSETTES: &'static str = "cassettes";

fn cassette(example: &str) -> PathBuf {
    Path::new(CASSETTES).join(example).with_extension("json")
}

fn run(example: &str) -> Vec<JsonFrame> {
    let string = String::from_utf8(
        Command::new("cargo")
            .args([
                "run",
                // "--target",
                // "thumbv6m-none-eabi",
                "--example",
                example,
            ])
            .output()
            .expect("failed to execute process")
            .stdout,
    )
    .unwrap();
    let lines = string.lines().collect::<Vec<_>>();
    let schema_version: SchemaVersion = serde_json::from_str(lines[0]).unwrap();
    lines[1..]
        .iter()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

fn record(example: &str) {
    let lines: Vec<String> = run(example)
        .into_iter()
        .map(|frame| format!("{:?}", frame))
        .collect();
    fs::write(cassette(example), lines.join("")).unwrap();
}

fn validate(output: &str) {}

fn main() {
    let mut args = args().skip(1);
    match (args.next(), args.next()) {
        (Some(taskname), Some(example)) if taskname == "record" => {
            record(&example);
        }
        (Some(taskname), Some(example)) if taskname == "validate" => {
            run(&example);
        }
        _ => {
            println!("usage information");
        }
    }
}
