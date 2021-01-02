use super::{HamiltonDriver, WireMoveCommand};
use anyhow::Result;
use async_trait::async_trait;
use lss_driver::LSSDriver;
use std::str;

pub struct HamiltonLssDriver {
    driver: LSSDriver,
}

impl HamiltonLssDriver {
    pub async fn new(port: &str) -> Result<Self> {
        let mut driver = LSSDriver::new(port)?;
        driver.set_maximum_speed(1, 360.0).await?;
        driver.set_maximum_speed(1, 360.0).await?;
        driver.set_maximum_speed(1, 360.0).await?;
        driver.set_maximum_speed(1, 360.0).await?;
        Ok(Self { driver })
    }
}

#[async_trait]
impl HamiltonDriver for HamiltonLssDriver {
    async fn send(&mut self, command: WireMoveCommand) -> Result<()> {
        self.driver
            .set_rotation_speed(1, command.wheel_a as f32)
            .await?;
        self.driver
            .set_rotation_speed(2, command.wheel_b as f32)
            .await?;
        self.driver
            .set_rotation_speed(3, command.wheel_c as f32)
            .await?;
        self.driver
            .set_rotation_speed(4, command.wheel_d as f32)
            .await?;
        Ok(())
    }
}
