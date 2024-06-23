use config::Config;
use serde::Deserialize;
use std::{path::PathBuf, str};
use tracing::*;

use crate::{driver::BodyConfig, error::ErrorWrapper, lidar::LidarConfig};

#[derive(Deserialize, Debug, Clone)]
pub struct AppConfig {
    pub body: BodyConfig,
    #[serde(default)]
    pub lidar: Option<LidarConfig>,
    #[serde(default)]
    pub zenoh: HamiltonZenohConfig,
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

#[derive(Deserialize, Debug, Clone, Default)]
pub struct HamiltonZenohConfig {
    #[serde(default)]
    pub connect: Vec<zenoh_config::EndPoint>,
    #[serde(default)]
    pub listen: Vec<zenoh_config::EndPoint>,
    #[serde(default)]
    pub config_path: Option<String>,
}

impl HamiltonZenohConfig {
    pub fn get_zenoh_config(&self) -> anyhow::Result<zenoh::config::Config> {
        let mut config = if let Some(conf_file) = &self.config_path {
            zenoh::config::Config::from_file(conf_file).map_err(ErrorWrapper::ZenohError)?
        } else {
            zenoh::config::Config::default()
        };
        if !self.connect.is_empty() {
            config.connect.endpoints.clone_from(&self.connect);
        }
        if !self.listen.is_empty() {
            config.listen.endpoints.clone_from(&self.listen);
        }
        Ok(config)
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
