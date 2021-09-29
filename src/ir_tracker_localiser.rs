use crate::navigation::Pose;
use anyhow::{anyhow, Result};
use na::distance;
use nalgebra as na;
use serde::{Deserialize, Serialize};
use socket2::{Domain, Protocol, Socket, Type};
use std::{
    net::{SocketAddrV4, UdpSocket as StdUdpSocket},
    str,
};
use tokio::{net::UdpSocket as TokioUdpSocket, sync::mpsc, task};
use tracing::{error, warn};

#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IrTrackers {
    frame_time: f32,
    point_count: usize,
    height: i32,
    width: i32,
    channels: i32,
    #[serde(rename = "useing_otsu_thresholding")]
    using_otsu_thresholding: bool,
    #[serde(rename = "binarization_threshold")]
    binarization_threshold: i32,
    points: Vec<na::Point2<f32>>,
}

fn linear_map(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

const TRIANGLE_REJECTION_DISTANCE: f32 = 0.02;

impl IrTrackers {
    fn points_in_screen_space(&self) -> Vec<na::Point2<f32>> {
        let mut new_points = Vec::with_capacity(self.point_count as usize);
        for point in &self.points {
            let new_x = linear_map(point.x, 0., self.width as f32, 0., 1.);
            let new_y = linear_map(point.y, 0., self.height as f32, 0., 1.);
            new_points.push(na::Point2::new(new_x, new_y));
        }
        new_points
    }

    pub fn find_tracker_pose(&self) -> Option<Pose> {
        if self.point_count < 4 {
            warn!("Less than 4 points visible");
            return None;
        }
        let local_points = self.points_in_screen_space();
        for current_point in &local_points {
            let mut points_copy = local_points.clone();
            points_copy = points_copy
                .into_iter()
                .filter(|other_point| other_point != current_point)
                .collect();
            if points_copy.len() < 3 {
                error!("Somehow ended up with less than 3 points");
                return None;
            }
            points_copy.sort_by(|a, b| {
                distance(current_point, a)
                    .partial_cmp(&distance(current_point, b))
                    .unwrap()
            });

            let first = points_copy.get(0).unwrap();
            let second = points_copy.get(1).unwrap();
            let distance_first = distance(current_point, first);
            let distance_second = distance(current_point, second);
            if distance_first > TRIANGLE_REJECTION_DISTANCE
                || distance_second > TRIANGLE_REJECTION_DISTANCE
            {
                // reject triangle point
                continue;
            }
            // we are in triangle now

            let triangle_center_x = (current_point.x + first.x + second.x) / 3.;
            let triangle_center_y = (current_point.y + first.y + second.y) / 3.;
            let triangle_center = na::Point2::new(triangle_center_x, triangle_center_y);

            let direction_point = points_copy.get(2).unwrap();

            let translation = direction_point.coords - triangle_center.coords;
            let normalized_translation = translation.normalize();
            let rotation =
                na::Rotation2::new(normalized_translation.x.atan2(normalized_translation.y));

            let triangle_to_robot_translation = na::Vector2::new(0.058, 0.0);
            // use hardcoded value because this causes jitter
            // Or maybe that's an over eager controller?
            // let triangle_to_robot_translation =
            //     na::Vector2::new(translation.magnitude() * 1.5, 0.0);

            let robot_pose = triangle_center + rotation * triangle_to_robot_translation;
            let pose = Pose::from_na(
                na::Point2::new(1.0 - robot_pose.y, 1.0 - robot_pose.x),
                rotation,
            );
            return Some(pose);
        }
        None
    }

    pub fn pretty_points(&self) -> String {
        let mut buffer = String::new();
        for point in &self.points_in_screen_space() {
            buffer.push_str(&format!("[{}, {}] ", point.x, point.y));
        }
        buffer
    }
}

fn bind_multicast(addr: &SocketAddrV4, multi_addr: &SocketAddrV4) -> Result<StdUdpSocket> {
    // this code was inspired by https://github.com/henninglive/tokio-udp-multicast-chat
    if !multi_addr.ip().is_multicast() {
        return Err(anyhow!("Address is not multicast"));
    }
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
) -> Result<mpsc::Receiver<IrTrackers>> {
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
                if let Ok(tracking_data) = serde_json::from_slice::<IrTrackers>(&buf[..len]) {
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
