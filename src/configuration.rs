use config::Config;
use serde::Deserialize;
use std::{path::PathBuf, str};
use tracing::*;

use crate::{driver::BodyConfig, lidar::LidarConfig};

#[derive(Deserialize, Debug, Clone)]
pub struct AppConfig {
    pub body: BodyConfig,
    #[serde(default)]
    pub lidar: Option<LidarConfig>,
}

impl AppConfig {
    pub fn load_config(config: &Option<PathBuf>) -> anyhow::Result<Self> {
        let settings = if let Some(config) = config {
            info!("Using configuration from {:?}", config);
            Config::builder()
                .add_source(config::Environment::with_prefix("APP"))
                .add_source(config::File::with_name(
                    config
                        .to_str()
                        .ok_or_else(|| anyhow::anyhow!("Failed to convert path"))?,
                ))
                .build()?
        } else {
            info!("Using dev configuration");
            Config::builder()
                .add_source(config::Environment::with_prefix("APP"))
                .add_source(config::File::with_name("config/settings"))
                .add_source(config::File::with_name("config/dev_settings"))
                .build()?
        };

        Ok(settings.try_deserialize()?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static DEFAULT_CONFIG: &str = include_str!("../config/settings.yaml");

    #[test]
    fn test_config() {
        let builder = Config::builder()
            .add_source(config::File::from_str(
                DEFAULT_CONFIG,
                config::FileFormat::Yaml,
            ))
            .build()
            .unwrap();
        builder.try_deserialize::<AppConfig>().unwrap();
    }
}
