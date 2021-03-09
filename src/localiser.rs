use anyhow::Result;
use nalgebra as na;
use serde::Deserialize;
use socket2::{Domain, Protocol, Socket, Type};
use std::{
    net::{SocketAddrV4, UdpSocket as StdUdpSocket},
    str,
};
use tokio::{net::UdpSocket as TokioUdpSocket, sync::mpsc, task};

#[derive(Debug, Deserialize)]
pub struct TrackedObjects {
    pub ts: u128,
    pub trackers: Vec<VrDevice>,
}

impl TrackedObjects {
    fn get_tracker_pose_in_openvr(&self) -> Option<(na::Point3<f32>, na::UnitQuaternion<f32>)> {
        for object in &self.trackers {
            if let VrDeviceClass::Tracker = object.class {
                if object.seen && object.tracked {
                    return Some((object.position, object.rotation));
                }
            }
        }
        None
    }

    pub fn get_tracker_pose(&self) -> Option<(na::Point2<f32>, f32)> {
        self.get_tracker_pose_in_openvr().map(tracker_pose_to_plane)
    }
}

pub fn tracker_pose_to_plane(
    pose: (na::Point3<f32>, na::UnitQuaternion<f32>),
) -> (na::Point2<f32>, f32) {
    let (position_openvr_space, rotation) = pose;
    let projection = rotation * na::Vector3::y_axis();
    let yaw = -projection.x.atan2(-projection.z);
    let position = position_openvr_space.zx();
    let position_centered = position + na::Rotation2::new(yaw) * na::Vector2::new(0.14, 0.0);
    (position_centered, yaw)
}

#[derive(Debug, Eq, PartialEq, Copy, Clone, Deserialize)]
pub enum VrDeviceClass {
    Controller,
    LeftController,
    RightController,
    Tracker,
    HMD,
    Sensor,
    Other,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VrDevice {
    pub id: usize,
    pub tracked: bool,
    pub seen: bool,
    pub position: na::Point3<f32>,
    pub rotation: na::UnitQuaternion<f32>,
    pub class: VrDeviceClass,
}

fn bind_multicast(addr: &SocketAddrV4, multi_addr: &SocketAddrV4) -> Result<StdUdpSocket> {
    // this code was inspired by https://github.com/henninglive/tokio-udp-multicast-chat
    assert!(multi_addr.ip().is_multicast(), "Address must be multicast");
    let socket = Socket::new(Domain::ipv4(), Type::dgram(), Some(Protocol::udp()))?;
    socket.set_reuse_address(true)?;
    socket.set_nonblocking(true)?;
    socket.bind(&socket2::SockAddr::from(*addr))?;
    socket.set_multicast_loop_v4(true)?;
    socket.join_multicast_v4(multi_addr.ip(), addr.ip())?;
    Ok(socket.into_udp_socket())
}

const IP_ALL: [u8; 4] = [0, 0, 0, 0];

pub async fn create_localization_subscriber(
    multi_addr: SocketAddrV4,
) -> Result<mpsc::Receiver<TrackedObjects>> {
    let (tx, rx) = mpsc::channel(10);
    let binding_addr = SocketAddrV4::new(IP_ALL.into(), multi_addr.port());
    let socket = bind_multicast(&binding_addr, &multi_addr)?;
    let socket = TokioUdpSocket::from_std(socket)?;
    task::spawn(async move {
        // we should never get messages this big. But since this buffer is simply allocated once
        // it doesn't cost us much to preallocate it
        let mut buf = [0; 65000];
        loop {
            // this logic is weirdly nested
            // may want to refactor this later
            if let Ok(len) = socket.recv(&mut buf).await {
                if let Ok(tracking_data) = serde_json::from_slice::<TrackedObjects>(&buf[..len]) {
                    if tx.send(tracking_data).await.is_err() {
                        eprintln!("Failed to send localization over channel");
                        return;
                    }
                } else {
                    eprintln!("Failed to parse json from localizer");
                }
            } else {
                eprintln!("Failed to read from UDP socket");
                return;
            }
        }
    });
    Ok(rx)
}
