pub mod hamilton_dc_driver;
pub mod hamilton_lss_driver;

use crate::holonomic_controller::HolonomicWheelCommand;
use anyhow::Result;
use async_trait::async_trait;
pub use hamilton_dc_driver::HamiltonDcDriver;
pub use hamilton_lss_driver::HamiltonLssDriver;
use lss_driver::LedColor;
use serde::{Deserialize, Serialize};
use std::{path::Path, sync::Arc};
use tokio::sync::Mutex;

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

pub async fn hamilton_driver_from_config(config: BodyConfig) -> Result<Box<dyn HamiltonDriver>> {
    if let DriverType::LSS = config.driver_type() {
        let lss_driver = Arc::new(Mutex::new(lss_driver::LSSDriver::new(&config.port)?));
        Ok(Box::new(HamiltonLssDriver::new(lss_driver, config).await?))
    } else {
        Ok(Box::new(HamiltonDcDriver::new(config)?))
    }
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

#[derive(Serialize, Deserialize, Debug, Clone, Copy, Default)]
pub enum DriverType {
    #[default]
    Arduino,
    LSS,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MotorConfig {
    id: u8,
    inverted: bool,
}

fn default_port() -> String {
    String::from("/dev/hamilton_dc_motors")
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BodyConfig {
    pub left_front_controller: MotorConfig,
    pub right_front_controller: MotorConfig,
    pub left_rear_controller: MotorConfig,
    pub right_rear_controller: MotorConfig,
    pub multiplier: f32,
    #[serde(default)]
    pub driver_type: DriverType,
    #[serde(default = "default_port")]
    pub port: String,
}

impl BodyConfig {
    pub fn load_json(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)?;
        let reader = std::io::BufReader::new(file);
        let parsed = serde_json::from_reader(reader)?;
        Ok(parsed)
    }

    pub fn save_json(&self, path: &Path) -> Result<()> {
        let file = std::fs::File::create(path)?;
        let writer = std::io::BufWriter::new(file);
        serde_json::to_writer_pretty(writer, self)?;
        Ok(())
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
