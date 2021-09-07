use super::{BodyConfig, Clampable, HamiltonDriver, MotorCommand};
use crate::{driver::MotorConfig, holonomic_controller::HolonomicWheelCommand};
use anyhow::Result;
use async_trait::async_trait;
use lss_driver::{LSSDriver, LedColor};
use std::sync::Arc;
use tokio::sync::Mutex;

pub struct HamiltonLssDriver {
    driver: Arc<Mutex<LSSDriver>>,
    config: BodyConfig,
}

impl HamiltonLssDriver {
    pub async fn new(driver: Arc<Mutex<LSSDriver>>, config: BodyConfig) -> Result<Self> {
        let mut driver_lock = driver.lock().await;
        for id in config.get_ids().iter() {
            driver_lock
                .set_maximum_speed(*id, config.multiplier.abs())
                .await?;
        }
        drop(driver_lock);
        Ok(Self { driver, config })
    }
}

#[async_trait]
impl HamiltonDriver for HamiltonLssDriver {
    async fn send(&mut self, command: HolonomicWheelCommand) -> Result<()> {
        let command = self.config.apply_commands_by_mapping(&command);
        let mut driver = self.driver.lock().await;
        for motor_command in command.motors() {
            driver
                .set_rotation_speed(motor_command.id(), motor_command.speed())
                .await?;
        }
        Ok(())
    }

    async fn read_voltage(&mut self) -> Result<Option<f32>> {
        let mut voltages = Vec::with_capacity(4);
        let mut driver = self.driver.lock().await;
        for id in self.config.get_ids().iter() {
            voltages.push(driver.query_voltage(*id).await?);
        }
        Ok(Some(voltages.iter().sum::<f32>() / voltages.len() as f32))
    }

    async fn set_color(&mut self, color: LedColor) -> Result<Option<()>> {
        let mut driver = self.driver.lock().await;
        for id in self.config.get_ids().iter() {
            driver.set_color(*id, color).await?;
        }
        Ok(Some(()))
    }
}

#[derive(Debug)]
pub struct LssWireMoveCommand {
    motors: [MotorCommand; 4],
}

impl LssWireMoveCommand {
    pub fn new(motors: [MotorCommand; 4]) -> Self {
        Self { motors }
    }

    pub fn motors(&self) -> &[MotorCommand; 4] {
        &self.motors
    }
}

trait ConfigMappable {
    fn apply_commands_by_mapping(&self, command: &HolonomicWheelCommand) -> LssWireMoveCommand;
}

impl ConfigMappable for BodyConfig {
    fn apply_commands_by_mapping(&self, command: &HolonomicWheelCommand) -> LssWireMoveCommand {
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
        LssWireMoveCommand::new([left_front, right_front, left_rear, right_rear])
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
