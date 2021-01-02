use crate::driver;
use anyhow::Result;
use directories::ProjectDirs;
use serde::{Deserialize, Serialize};
use std::fs::create_dir_all;
use std::path::Path;

use crate::hamilton::MoveCommand;

trait Clampable {
    fn clamp_num(&self, a: Self, b: Self) -> Self;
}

impl Clampable for i32 {
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

pub struct HolonomicWheelCommand {
    left_front: f32,
    right_front: f32,
    left_rear: f32,
    right_rear: f32,
}

impl HolonomicWheelCommand {
    pub fn new(
        left_front: f32,
        right_front: f32,
        left_rear: f32,
        right_rear: f32,
    ) -> HolonomicWheelCommand {
        HolonomicWheelCommand {
            left_front,
            right_front,
            left_rear,
            right_rear,
        }
    }
}

impl From<MoveCommand> for HolonomicWheelCommand {
    fn from(move_command: MoveCommand) -> Self {
        let forward = move_command.x;
        let strafe = move_command.y;
        let rotation = move_command.yaw;
        HolonomicWheelCommand::new(
            forward - rotation - strafe,
            forward + rotation + strafe,
            forward - rotation + strafe,
            forward + rotation - strafe,
        )
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub enum MotorMappingFlags {
    A(bool),
    B(bool),
    C(bool),
    D(bool),
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MotorMapping {
    pub left_front_controller: MotorMappingFlags,
    pub right_front_controller: MotorMappingFlags,
    pub left_rear_controller: MotorMappingFlags,
    pub right_rear_controller: MotorMappingFlags,
    pub multiplier: f32,
}

impl MotorMapping {
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

    pub fn apply_commands_by_mapping(
        &self,
        command: &HolonomicWheelCommand,
    ) -> driver::WireMoveCommand {
        fn apply_motor(
            wire_command: &mut driver::WireMoveCommand,
            mapping: &MotorMappingFlags,
            value: i32,
        ) {
            match mapping {
                MotorMappingFlags::A(reversed) if *reversed => wire_command.wheel_a = -value,
                MotorMappingFlags::A(_) => wire_command.wheel_a = value,
                MotorMappingFlags::B(reversed) if *reversed => wire_command.wheel_b = -value,
                MotorMappingFlags::B(_) => wire_command.wheel_b = value,
                MotorMappingFlags::C(reversed) if *reversed => wire_command.wheel_c = -value,
                MotorMappingFlags::C(_) => wire_command.wheel_c = value,
                MotorMappingFlags::D(reversed) if *reversed => wire_command.wheel_d = -value,
                MotorMappingFlags::D(_) => wire_command.wheel_d = value,
            }
        }
        let mut wire_command = driver::WireMoveCommand::default();
        let clamp_range = self.multiplier as i32;
        apply_motor(
            &mut wire_command,
            &self.left_front_controller,
            ((command.left_front * self.multiplier) as i32).clamp_num(-clamp_range, clamp_range),
        );
        apply_motor(
            &mut wire_command,
            &self.right_front_controller,
            ((command.right_front * self.multiplier) as i32).clamp_num(-clamp_range, clamp_range),
        );
        apply_motor(
            &mut wire_command,
            &self.left_rear_controller,
            ((command.left_rear * self.multiplier) as i32).clamp_num(-clamp_range, clamp_range),
        );
        apply_motor(
            &mut wire_command,
            &self.right_rear_controller,
            ((command.right_rear * self.multiplier) as i32).clamp_num(-clamp_range, clamp_range),
        );
        wire_command
    }
}

impl Default for MotorMapping {
    fn default() -> Self {
        MotorMapping {
            left_front_controller: MotorMappingFlags::A(false),
            right_front_controller: MotorMappingFlags::B(false),
            left_rear_controller: MotorMappingFlags::C(false),
            right_rear_controller: MotorMappingFlags::D(false),
            multiplier: 255.,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_mapping_checks() {
        let mapping = MotorMapping::default();
        let holo_wheel_command = HolonomicWheelCommand::new(1.0, 0.8, 0.6, 0.4);
        let wire_command = mapping.apply_commands_by_mapping(&holo_wheel_command);
        assert_eq!(wire_command.wheel_a, 255);
        assert_eq!(wire_command.wheel_b, 204);
        assert_eq!(wire_command.wheel_c, 153);
        assert_eq!(wire_command.wheel_d, 102);
    }

    #[test]
    fn mapping_checks_reversed() {
        let mapping = MotorMapping {
            right_front_controller: MotorMappingFlags::B(true),
            ..Default::default()
        };
        let holo_wheel_command = HolonomicWheelCommand::new(1.0, 0.8, 0.6, 0.4);
        let wire_command = mapping.apply_commands_by_mapping(&holo_wheel_command);
        assert_eq!(wire_command.wheel_a, 255);
        assert_eq!(wire_command.wheel_b, -204);
        assert_eq!(wire_command.wheel_c, 153);
        assert_eq!(wire_command.wheel_d, 102);
    }

    #[test]
    fn mapping_checks_swapped() {
        let mapping = MotorMapping {
            right_front_controller: MotorMappingFlags::C(false),
            left_rear_controller: MotorMappingFlags::B(false),
            ..Default::default()
        };
        let holo_wheel_command = HolonomicWheelCommand::new(1.0, 0.8, 0.6, 0.4);
        let wire_command = mapping.apply_commands_by_mapping(&holo_wheel_command);
        assert_eq!(wire_command.wheel_a, 255);
        assert_eq!(wire_command.wheel_b, 153);
        assert_eq!(wire_command.wheel_c, 204);
        assert_eq!(wire_command.wheel_d, 102);
    }

    #[test]
    fn default_mapping_multiplier() {
        let mapping = MotorMapping::default();
        let holo_wheel_command = HolonomicWheelCommand::new(1., 0.5, 1., 0.8);
        let wire_command = mapping.apply_commands_by_mapping(&holo_wheel_command);
        assert_eq!(wire_command.wheel_a, 255);
        assert_eq!(wire_command.wheel_b, 127);
        assert_eq!(wire_command.wheel_c, 255);
        assert_eq!(wire_command.wheel_d, 204);
    }

    #[test]
    fn default_mapping_clamping() {
        let mapping = MotorMapping::default();
        let holo_wheel_command = HolonomicWheelCommand::new(2., 2.5, -2.6, -7.);
        let wire_command = mapping.apply_commands_by_mapping(&holo_wheel_command);
        assert_eq!(wire_command.wheel_a, 255);
        assert_eq!(wire_command.wheel_b, 255);
        assert_eq!(wire_command.wheel_c, -255);
        assert_eq!(wire_command.wheel_d, -255);
    }

    #[test]
    fn clamp_max() {
        let input = 200;
        let clamped = input.clamp_num(100, -100);
        assert_eq!(clamped, 100);
    }

    #[test]
    fn clamp_max_reversed() {
        let input = 200;
        let clamped = input.clamp_num(-100, 100);
        assert_eq!(clamped, 100);
    }

    #[test]
    fn clamp_min() {
        let input = -200;
        let clamped = input.clamp_num(-100, 100);
        assert_eq!(clamped, -100);
    }

    #[test]
    fn clamp_min_reversed() {
        let input = -200;
        let clamped = input.clamp_num(100, -100);
        assert_eq!(clamped, -100);
    }
}
