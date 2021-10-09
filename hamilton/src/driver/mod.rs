pub mod hamilton_dc_driver;
pub mod hamilton_lss_driver;

use std::{fs::create_dir_all, path::Path};

use crate::holonomic_controller::HolonomicWheelCommand;
use anyhow::Result;
use async_trait::async_trait;
use directories::ProjectDirs;
pub use hamilton_dc_driver::HamiltonDcDriver;
pub use hamilton_lss_driver::HamiltonLssDriver;
use lss_driver::LedColor;
use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub struct MotorCommand {
    id: u8,
    speed: f32,
}

impl MotorCommand {
    pub fn new(id: u8, speed: f32) -> Self {
        Self { id, speed }
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    pub fn speed(&self) -> f32 {
        self.speed
    }
}

#[async_trait]
pub trait HamiltonDriver: Send {
    async fn send(&mut self, command: HolonomicWheelCommand) -> Result<()>;
    async fn read_voltage(&mut self) -> Result<Option<f32>>;
    async fn set_color(&mut self, color: LedColor) -> Result<Option<()>>;
    fn set_halt_mode(&mut self, on: bool);
    fn halt_mode(&self) -> bool;
}

trait Clampable {
    fn clamp_num(&self, a: Self, b: Self) -> Self;
}

impl Clampable for f32 {
    fn clamp_num(&self, a: Self, b: Self) -> Self {
        let min = a.min(b);
        let max = a.max(b);
        if *self > max {
            max
        } else if *self < min {
            min
        } else {
            *self
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum DriverType {
    Arduino,
    LSS,
}

impl Default for DriverType {
    fn default() -> DriverType {
        DriverType::LSS
    }
}

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct MotorConfig {
    id: u8,
    inverted: bool,
}

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct BodyConfig {
    pub left_front_controller: MotorConfig,
    pub right_front_controller: MotorConfig,
    pub left_rear_controller: MotorConfig,
    pub right_rear_controller: MotorConfig,
    pub multiplier: f32,
    pub driver_type: DriverType,
}

impl BodyConfig {
    pub fn load(path: &str) -> Result<Self> {
        let file = std::fs::File::open(path)?;
        let reader = std::io::BufReader::new(file);
        let parsed = serde_json::from_reader(reader)?;
        Ok(parsed)
    }

    pub fn save(&self, path: &str) -> Result<()> {
        let file = std::fs::File::create(path)?;
        let writer = std::io::BufWriter::new(file);
        serde_json::to_writer_pretty(writer, self)?;
        Ok(())
    }

    /// Try to load config from default system location
    ///
    /// If not present will write default config there
    pub fn load_from_default() -> Result<Self> {
        // TODO: This functionality may not belong here but whatever
        if let Some(project_dirs) = ProjectDirs::from("com", "David Weis", "Hamilton controller") {
            let config_dir = project_dirs.config_dir();
            create_dir_all(config_dir)?;
            let config_path = Path::new(config_dir).join("wheel_config.json");
            let config_path = config_path.to_str().unwrap();
            if let Ok(config) = Self::load(config_path) {
                Ok(config)
            } else {
                let default = Self::default();
                default.save_to_default()?;
                Ok(default)
            }
        } else {
            panic!("Failed to find system specific config folder\nIs $HOME not defined?");
        }
    }

    /// Save to default location
    pub fn save_to_default(&self) -> Result<()> {
        // TODO: This method duplicated a lot from load_from_default
        if let Some(project_dirs) = ProjectDirs::from("com", "David Weis", "Hamilton controller") {
            let config_dir = project_dirs.config_dir();
            create_dir_all(config_dir)?;
            let config_path = Path::new(config_dir).join("wheel_config.json");
            let config_path = config_path.to_str().unwrap();
            Self::default().save(config_path)?;
            Ok(())
        } else {
            panic!("Failed to find system specific config folder\nIs $HOME not defined?");
        }
    }

    pub fn get_ids(&self) -> [u8; 4] {
        [
            self.left_front_controller.id,
            self.right_front_controller.id,
            self.left_rear_controller.id,
            self.right_rear_controller.id,
        ]
    }

    pub fn driver_type(&self) -> DriverType {
        self.driver_type
    }
}
