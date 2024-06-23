use anyhow::Result;
use clap::Parser;
use hamilton::{configuration, driver::hamilton_driver_from_config, lidar::Lidar, logging};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(
    version,
    author = "David M. Weis <dweis7@gmail.com>",
    about = "Hamilton"
)]
struct Args {
    /// path to config
    #[arg(long)]
    config: Option<PathBuf>,

    /// Sets the level of verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbosity: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    logging::setup_tracing(args.verbosity);

    let app_config = configuration::AppConfig::load_config(&args.config)?;

    let body_config = app_config.body.clone();

    if let Some(lidar_config) = &app_config.lidar {
        let _lidar_driver = Lidar::open(lidar_config.clone())?;
    }

    let _driver = hamilton_driver_from_config(body_config).await?;

    Ok(())
}
