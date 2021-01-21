pub mod hamilton_lss_driver;

use anyhow::Result;
use async_trait::async_trait;
use lss_driver::LedColor;

#[derive(Default, Debug)]
pub struct WireMoveCommand {
    pub wheel_a: f32,
    pub wheel_b: f32,
    pub wheel_c: f32,
    pub wheel_d: f32,
}

#[async_trait]
pub trait HamiltonDriver: Send + Sync {
    async fn send(&mut self, command: WireMoveCommand) -> Result<()>;
    async fn read_voltage(&mut self) -> Result<Option<f32>>;
    async fn set_color(&mut self, color: LedColor) -> Result<()>;
}
