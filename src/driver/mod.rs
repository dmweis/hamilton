pub mod hamilton_lss_driver;

pub use hamilton_lss_driver::HamiltonLssDriver;

#[derive(Default, Debug)]
pub struct WireMoveCommand {
    pub wheel_a: f32,
    pub wheel_b: f32,
    pub wheel_c: f32,
    pub wheel_d: f32,
}
