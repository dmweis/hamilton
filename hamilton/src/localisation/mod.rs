pub mod ir_tracker_localiser;
pub mod openvr_localiser;

use crate::navigation::Pose2d;

use self::ir_tracker_localiser::IrTrackers;
use anyhow::Result;
use std::{
    net::SocketAddrV4,
    time::{Duration, Instant},
};
use tokio::sync::mpsc::{error::TryRecvError, Receiver};

pub enum LocaliserType {
    IrMarker,
    OpenVr,
}

static LOCALISATION_TIMEOUT: Duration = Duration::from_secs(1);

pub struct LocalisationManager {
    localisation_receiver: Receiver<IrTrackers>,
    last_update_time: Instant,
    last_pose: Pose2d,
}

impl LocalisationManager {
    pub async fn new(topic_addr: SocketAddrV4, localiser_type: LocaliserType) -> Result<Self> {
        match localiser_type {
            LocaliserType::IrMarker => {
                let localisation_receiver =
                    ir_tracker_localiser::create_localization_subscriber(topic_addr).await?;
                Ok(Self {
                    localisation_receiver,
                    // Maybe these should be optional with None?
                    last_pose: Pose2d::new((0., 0.), 0.),
                    last_update_time: Instant::now() - LOCALISATION_TIMEOUT,
                })
            }
            LocaliserType::OpenVr => todo!("OpenVR localiser not implemented"),
        }
    }

    pub async fn get_latest_pose(&mut self) -> Result<Option<Pose2d>> {
        loop {
            let message = self.localisation_receiver.try_recv();
            match message {
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    return Err(anyhow::anyhow!("Localisation channel disconnected"))
                }
                Ok(trackers) => {
                    if let Some(pose) = trackers.find_tracker_pose() {
                        self.last_pose = pose;
                        self.last_update_time = Instant::now();
                    }
                }
            }
        }
        if self.last_update_time.elapsed() > LOCALISATION_TIMEOUT {
            Ok(None)
        } else {
            Ok(Some(self.last_pose.clone()))
        }
    }
}
