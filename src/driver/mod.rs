pub mod hamilton_lss_driver;

use crate::holonomic_controller::HolonomicWheelCommand;
use anyhow::Result;
use async_trait::async_trait;
pub use hamilton_lss_driver::{BodyConfig, HamiltonLssDriver};
use lss_driver::LedColor;

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
pub trait HamiltonDriver: Send + Sync {
    async fn send(&mut self, command: HolonomicWheelCommand) -> Result<()>;
    async fn read_voltage(&mut self) -> Result<Option<f32>>;
    async fn set_color(&mut self, color: LedColor) -> Result<Option<()>>;
}
