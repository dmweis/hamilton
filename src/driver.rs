use anyhow::Result;
use lss_driver::LSSDriver;
use std::str;

#[derive(Default, Debug)]
pub struct WireMoveCommand {
    pub wheel_a: i32,
    pub wheel_b: i32,
    pub wheel_c: i32,
    pub wheel_d: i32,
}

pub struct HamiltonDriver {
    driver: LSSDriver,
}

impl HamiltonDriver {
    pub fn new(port: &str) -> Result<HamiltonDriver> {
        let driver = LSSDriver::new(port)?;
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
