use anyhow::Result;
use lss_driver::LSSDriver;
use std::str;

#[derive(Default, Debug)]
pub struct WireMoveCommand {
    pub wheel_a: f32,
    pub wheel_b: f32,
    pub wheel_c: f32,
    pub wheel_d: f32,
}

pub struct HamiltonDriver {
    driver: LSSDriver,
}

impl HamiltonDriver {
    pub async fn new(port: &str) -> Result<HamiltonDriver> {
        let mut driver = LSSDriver::new(port)?;
        driver.set_maximum_speed(1, 50.0 * 360.0).await?;
        driver.set_maximum_speed(1, 50.0 * 360.0).await?;
        driver.set_maximum_speed(1, 50.0 * 360.0).await?;
        driver.set_maximum_speed(1, 50.0 * 360.0).await?;
        Ok(HamiltonDriver { driver })
    }

    pub async fn send(&mut self, command: WireMoveCommand) -> Result<()> {
        println!("Writing {:?}", command);
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
