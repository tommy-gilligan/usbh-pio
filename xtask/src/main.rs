use std::process::Command;
use std::env::args;
use std::path::{Path, PathBuf};
use std::fs::File;
use std::fs;

const CASSETTES: &'static str = "cassettes";

fn cassette(example: &str) -> PathBuf {
    Path::new(CASSETTES).join(example).with_extension("json")
}

fn run(example: &str) -> Vec<u8> {
    Command::new("cargo")
        .args(
            [
                "run",
                // "--target",
                // "thumbv6m-none-eabi",
                "--example",
                example
            ]
        )
        .output()
        .expect("failed to execute process")
        .stdout
}

fn record(example: &str) {
    fs::write(cassette(example), run(example)).unwrap();
}

fn validate(output: &str) {
}

fn main() {
    let mut args = args().skip(1);
    match (args.next(), args.next()) {
        (Some(taskname), Some(example)) if taskname == "record" => {
            record(&example);
        },
        (Some(taskname), Some(example)) if taskname == "validate" => {
            run(&example);
        },
        _ => {
            println!("usage information");
        }
    }
}
