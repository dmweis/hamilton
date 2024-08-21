use anyhow::Result;
use clap::Parser;
use hamilton::{
    configuration, driver::hamilton_driver_from_config, error::ErrorWrapper,
    gamepad::start_gamepad_loop, ioc::IocContainer, lidar::Lidar, logging,
};
use std::path::PathBuf;
use zenoh::prelude::r#async::*;

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

    let driver = hamilton_driver_from_config(body_config).await?;

    // zenoh
    let zenoh_config = app_config.zenoh.get_zenoh_config()?;
    let zenoh_session = zenoh::open(zenoh_config)
        .res()
        .await
        .map_err(ErrorWrapper::ZenohError)?
        .into_arc();

    let ioc_container = IocContainer::global_instance();
    ioc_container.register_arc(zenoh_session.clone());

    start_gamepad_loop(zenoh_session, driver).await?;

    tokio::signal::ctrl_c().await?;

    Ok(())
}
