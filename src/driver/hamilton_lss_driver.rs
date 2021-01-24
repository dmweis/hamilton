use crate::holonomic_controller::HolonomicWheelCommand;

use super::MotorCommand;
use anyhow::Result;
use directories::ProjectDirs;
use lss_driver::{LSSDriver, LedColor};
use serde::{Deserialize, Serialize};
use std::fs::create_dir_all;
use std::path::Path;
use std::str;

pub struct HamiltonLssDriver {
    driver: LSSDriver,
    config: BodyConfig,
}

impl HamiltonLssDriver {
    pub async fn new(port: &str, config: BodyConfig) -> Result<Self> {
        let mut driver = LSSDriver::new(port)?;
        for id in config.get_ids().iter() {
            driver
                .set_maximum_speed(*id, config.multiplier.abs())
                .await?;
        }
        Ok(Self { driver, config })
    }

    pub async fn send(&mut self, command: HolonomicWheelCommand) -> Result<()> {
        let command = self.config.apply_commands_by_mapping(&command);
        for motor_command in command.motors() {
            self.driver
                .set_rotation_speed(motor_command.id(), motor_command.speed())
                .await?;
        }
        Ok(())
    }

    pub async fn read_voltage(&mut self) -> Result<f32> {
        let mut voltages = Vec::with_capacity(4);
        for id in self.config.get_ids().iter() {
            voltages.push(self.driver.query_voltage(*id).await?);
        }
        Ok(voltages.iter().sum::<f32>() / voltages.len() as f32)
    }

    pub async fn set_color(&mut self, color: LedColor) -> Result<()> {
        for id in self.config.get_ids().iter() {
            self.driver.set_color(*id, color).await?;
        }
        Ok(())
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

    pub fn apply_commands_by_mapping(&self, command: &HolonomicWheelCommand) -> WireMoveCommand {
        fn create_motor_data(mapping: &MotorConfig, value: f32) -> MotorCommand {
            if mapping.inverted {
                MotorCommand::new(mapping.id, value * -1.0)
            } else {
                MotorCommand::new(mapping.id, value)
            }
        }
        let clamp_range = self.multiplier;
        let left_front = create_motor_data(
            &self.left_front_controller,
            (command.left_front() * self.multiplier).clamp_num(-clamp_range, clamp_range),
        );
        let right_front = create_motor_data(
            &self.right_front_controller,
            (command.right_front() * self.multiplier).clamp_num(-clamp_range, clamp_range),
        );
        let left_rear = create_motor_data(
            &self.left_rear_controller,
            (command.left_rear() * self.multiplier).clamp_num(-clamp_range, clamp_range),
        );
        let right_rear = create_motor_data(
            &self.right_rear_controller,
            (command.right_rear() * self.multiplier).clamp_num(-clamp_range, clamp_range),
        );
        WireMoveCommand::new([left_front, right_front, left_rear, right_rear])
    }
}

#[derive(Debug)]
pub struct WireMoveCommand {
    motors: [MotorCommand; 4],
}

impl WireMoveCommand {
    pub fn new(motors: [MotorCommand; 4]) -> Self {
        Self { motors }
    }

    pub fn motors(&self) -> &[MotorCommand; 4] {
        &self.motors
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn clamp_max() {
        let input = 200.0;
        let clamped = input.clamp_num(100.0, -100.0);
        assert_relative_eq!(clamped, 100.0);
    }

    #[test]
    fn clamp_max_reversed() {
        let input = 200.0;
        let clamped = input.clamp_num(-100.0, 100.0);
        assert_relative_eq!(clamped, 100.0);
    }

    #[test]
    fn clamp_min() {
        let input = -200.0;
        let clamped = input.clamp_num(-100.0, 100.0);
        assert_relative_eq!(clamped, -100.0);
    }

    #[test]
    fn clamp_min_reversed() {
        let input = -200.0;
        let clamped = input.clamp_num(100.0, -100.0);
        assert_relative_eq!(clamped, -100.0);
    }
}
