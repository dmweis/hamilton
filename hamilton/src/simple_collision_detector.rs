use crate::{holonomic_controller::MoveCommand, lidar::Lidar};
use rplidar_driver::ScanPoint;

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

const SAFE_DISTANCE: f32 = 0.3;
const SCAN_AREA: f32 = std::f32::consts::FRAC_PI_4;

fn collision_check(scan: &[ScanPoint], command: &MoveCommand) -> bool {
    // This isn't actually correct.... just inverted because the lidar is I think
    let move_direction_lidar = command.strafe().atan2(-command.forward()) + std::f32::consts::PI;

    scan.iter()
        .filter(|point| point.is_valid())
        .filter(|point| point.angle() > move_direction_lidar - SCAN_AREA)
        .filter(|point| point.angle() < move_direction_lidar + SCAN_AREA)
        .all(|point| point.distance() > SAFE_DISTANCE)
}
