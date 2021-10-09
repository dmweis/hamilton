use anyhow::Result;
use rplidar_driver::{RplidarDevice, RplidarDriver, ScanOptions};

pub struct Lidar {
    driver: Box<dyn RplidarDriver>,
}

impl Lidar {
    pub fn open(port: &str) -> Result<Self> {
        let mut lidar = RplidarDevice::open_port(port)?;
        let scan_options = ScanOptions::with_mode(2);
        let _ = lidar.start_scan_with_options(&scan_options)?;
        Ok(Self { driver: lidar })
    }

    pub fn stop_motor(&mut self) -> Result<()> {
        self.driver.stop_motor()?;
        Ok(())
    }
}
