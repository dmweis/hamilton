use anyhow::Result;
use rplidar_driver::{utils::sort_scan, RplidarDevice, RplidarDriver, ScanOptions, ScanPoint};
use serde::Deserialize;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::spawn,
    time::{Duration, Instant},
};
use tracing::*;

type LidarScan = (Vec<ScanPoint>, Instant);

fn default_lidar_port() -> String {
    String::from("/dev/rplidar")
}

#[derive(Deserialize, Debug, Clone)]
pub struct LidarConfig {
    #[serde(default = "default_lidar_port")]
    pub port: String,
}

pub struct Lidar {
    should_exit: Arc<AtomicBool>,
    should_spin: Arc<AtomicBool>,
    // bad david using dumb locking
    last_scan: Arc<Mutex<Option<LidarScan>>>,
}

const SCAN_TIMEOUT: Duration = Duration::from_millis(500);

impl Lidar {
    pub fn open(config: LidarConfig) -> Result<Self> {
        let last_scan = Arc::default();
        let should_exit = Arc::new(AtomicBool::new(false));
        let should_spin = Arc::new(AtomicBool::new(true));
        spawn({
            let last_scan = Arc::clone(&last_scan);
            let should_exit = Arc::clone(&should_exit);
            let should_spin = Arc::clone(&should_spin);

            move || run_lidar_loop(config.port, should_exit, should_spin, last_scan)
        });
        Ok(Self {
            should_exit,
            should_spin,
            last_scan,
        })
    }

    pub fn stop_motor(&mut self) {
        self.should_spin.store(false, Ordering::SeqCst);
    }

    pub fn start_motor(&mut self) {
        self.should_spin.store(true, Ordering::SeqCst);
    }

    pub fn get_last_scan(&self) -> Option<Vec<ScanPoint>> {
        let lock = self.last_scan.lock().unwrap();
        if let Some((scan, time)) = &*lock {
            if time.elapsed() < SCAN_TIMEOUT {
                Some(scan.clone())
            } else {
                None
            }
        } else {
            None
        }
    }
}

impl Drop for Lidar {
    fn drop(&mut self) {
        self.should_spin.store(false, Ordering::SeqCst);
        self.should_exit.store(true, Ordering::SeqCst);
    }
}

fn run_lidar_loop(
    port: String,
    exit_loop: Arc<AtomicBool>,
    spin: Arc<AtomicBool>,
    last_scan: Arc<Mutex<Option<LidarScan>>>,
) {
    while !exit_loop.load(Ordering::SeqCst) {
        info!("Starting lidar task");
        if let Ok(lidar) = RplidarDevice::open_port(&port) {
            if inner_lidar_loop(lidar, exit_loop.clone(), spin.clone(), last_scan.clone()).is_err()
            {
                error!("Lidar task failed");
            }
        } else {
            error!("Failed to open lidar")
        }
    }
}

fn inner_lidar_loop(
    mut lidar: Box<dyn RplidarDriver>,
    exit_loop: Arc<AtomicBool>,
    spin: Arc<AtomicBool>,
    last_scan: Arc<Mutex<Option<LidarScan>>>,
) -> Result<()> {
    let scan_options = ScanOptions::with_mode(2);
    let _ = lidar.start_scan_with_options(&scan_options)?;
    let mut is_spinning = true;
    while !exit_loop.load(Ordering::SeqCst) {
        let should_spin = spin.load(Ordering::SeqCst);
        if should_spin {
            if !is_spinning {
                is_spinning = true;
                lidar.start_motor()?;
                let scan_options = ScanOptions::with_mode(2);
                let _ = lidar.start_scan_with_options(&scan_options)?;
            }
            match lidar.grab_scan() {
                Ok(mut scan) => {
                    sort_scan(&mut scan)?;
                    last_scan
                        .lock()
                        .unwrap()
                        .replace((scan.clone(), Instant::now()));

                    let _scan_points = scan
                        .into_iter()
                        .filter(|scan| scan.is_valid())
                        .map(|scan_point| {
                            let x = scan_point.distance() * (-scan_point.angle()).cos();
                            let y = scan_point.distance() * (-scan_point.angle()).sin();
                            (x, y)
                        })
                        .collect::<Vec<_>>();
                }
                Err(rplidar_driver::RposError::OperationTimeout) => (),
                Err(error) => {
                    error!("Lidar error detected {:?}", error);
                    return Err(anyhow::anyhow!("Lidar error"));
                }
            }
        } else {
            // shouldn't spin
            if is_spinning {
                is_spinning = false;
                lidar.stop_motor()?;
                lidar.stop()?;
            }
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    }
    Ok(())
}
