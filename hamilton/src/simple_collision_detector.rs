use crate::{holonomic_controller::MoveCommand, lidar::Lidar};
use rplidar_driver::ScanPoint;
use tracing::*;

pub struct SimpleCollisionDetector {
    lidar: Lidar,
}

impl SimpleCollisionDetector {
    pub fn new(lidar: Lidar) -> Self {
        Self { lidar }
    }

    /// This is the most primitive collider possible
    pub fn check_move_safe(&mut self, command: &MoveCommand) -> bool {
        match self.lidar.get_last_scan() {
            Some(scan) => collision_check(&scan, command),
            None => true,
        }
    }

    pub fn start_lidar(&mut self) {
        self.lidar.start_motor();
    }

    pub fn stop_lidar(&mut self) {
        self.lidar.stop_motor()
    }
}

fn collision_check(_scan: &Vec<ScanPoint>, command: &MoveCommand) -> bool {
    let move_direction = command.forward().atan2(command.strafe()).to_degrees();
    info!("Moving in direction {}", move_direction);
    false
}
