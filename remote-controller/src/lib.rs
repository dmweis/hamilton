#![doc = include_str!("../README.md")]

mod controller_service;

pub use controller_service::*;
use futures::{SinkExt, StreamExt};
use include_dir::{include_dir, Dir};
use serde::Serialize;
use std::{sync::Arc, time::Duration};
use tokio::time::{sleep, timeout};
use tracing::*;
use warp::{
    filters::BoxedFilter,
    hyper::StatusCode,
    ws::{Message, WebSocket},
    Filter, Reply,
};

const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(5);
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Debug, Serialize, Clone, Copy)]
pub struct AreaSize {
    pub width: f32,
    pub height: f32,
}

impl AreaSize {
    pub fn new(width: f32, height: f32) -> Self {
        Self { width, height }
    }
}

type SharedControllerState = Arc<ControllerState>;

async fn handle_websocket(ws: WebSocket, controller_state: SharedControllerState) {
    trace!("new websocket connection");
    let (mut ws_tx, mut ws_rx) = ws.split();

    tokio::task::spawn(async move {
        loop {
            sleep(HEARTBEAT_INTERVAL).await;
            if ws_tx.send(Message::ping("")).await.is_err() {
                error!("Failed to send ping");
                break;
            }
        }
    });

    while let Ok(Some(result)) = timeout(CLIENT_TIMEOUT, ws_rx.next()).await {
        match result {
            Ok(msg) => {
                if let Ok(text) = msg.to_str() {
                    if let Ok(command) = serde_json::from_str(text) {
                        controller_state.set_gamepad(command).await;
                    } else {
                        error!("Failed to parse json {}", text);
                    }
                } else if msg.is_pong() {
                    trace!("Pong received");
                } else if msg.is_close() {
                    trace!("Got closing message. Closing connection");
                } else {
                    error!("Unknown message type {:?}", msg);
                }
            }
            Err(e) => {
                error!("websocket error: {}", e);
                break;
            }
        }
    }

    error!("User connection ended");
}

const STATIC_FILES_DIR: Dir = include_dir!("static");

pub fn start_remote_controller_server(
    address: impl Into<std::net::SocketAddr>,
) -> SharedControllerState {
    start_remote_controller_server_with_map(address, AreaSize::new(1.0, 1.0), ActionList::default())
}

pub fn start_remote_controller_server_with_map(
    address: impl Into<std::net::SocketAddr>,
    map_size: AreaSize,
    actions: ActionList,
) -> SharedControllerState {
    let address = address.into();

    let shared_controller_state = SharedControllerState::default();
    let shared_controller_state_copy = shared_controller_state.clone();
    let shared_controller_state_wrapper =
        warp::any().map(move || shared_controller_state_copy.clone());

    let ws = warp::path("ws")
        .and(warp::ws())
        .and(shared_controller_state_wrapper.clone())
        .map(|ws: warp::ws::Ws, controller| {
            ws.on_upgrade(move |socket| handle_websocket(socket, controller))
        });

    let map_size_endpoint = warp::path("map").map(move || warp::reply::json(&map_size));

    let canvas_touch_endpoint = warp::path("canvas_touch")
        .and(warp::filters::body::json())
        .and(shared_controller_state_wrapper.clone())
        .map(
            |data: CanvasTouch, controller_state: SharedControllerState| {
                controller_state.send_canvas_touch(data).unwrap();
                warp::reply()
            },
        );

    let actions_endpoint = warp::path("actions").map(move || warp::reply::json(&actions));
    let action_submit_endpoint = warp::path("action")
        .and(warp::post())
        .and(warp::filters::body::json())
        .and(shared_controller_state_wrapper)
        .map(
            |action: ActionIdWrapper, shared_controller: SharedControllerState| {
                shared_controller.send_action(action).unwrap();
                warp::reply()
            },
        );

    let index = warp::path::end().map(|| warp::reply::html(include_str!("../static/index.html")));

    let nipple_js_lib = warp::path("nipplejs.min.js")
        .map(|| warp::reply::html(include_str!(concat!(env!("OUT_DIR"), "/nipplejs.min.js"))));

    let static_file = static_file_route();

    let routes = index
        .or(ws)
        .or(canvas_touch_endpoint)
        .or(actions_endpoint)
        .or(action_submit_endpoint)
        .or(map_size_endpoint)
        .or(nipple_js_lib)
        .or(static_file);

    tokio::task::spawn(async move {
        warp::serve(routes).run(address).await;
    });

    shared_controller_state
}

fn static_file_route() -> BoxedFilter<(impl Reply,)> {
    warp::path("static")
        .and(warp::path::param())
        .map(|param: String| -> Box<dyn warp::reply::Reply> { get_static_file(&param) })
        .boxed()
}

fn get_static_file(path: &str) -> Box<dyn Reply> {
    STATIC_FILES_DIR
        .get_file(&path)
        .and_then(|file| file.contents_utf8().map(|contents| (contents, path)))
        .map(|(file_text, path)| -> Box<dyn Reply> {
            // Manually checking file extensions...
            // I am not good at this web stuff
            if path.ends_with(".html") || path.ends_with(".js") {
                Box::new(warp::reply::html(file_text))
            } else if path.ends_with(".css") {
                Box::new(warp::reply::with_header(
                    file_text,
                    "Content-Type",
                    "text/css",
                ))
            } else {
                Box::new(warp::reply::with_status(
                    "",
                    StatusCode::INTERNAL_SERVER_ERROR,
                ))
            }
        })
        .unwrap_or_else(|| Box::new(warp::reply::with_status("", StatusCode::NOT_FOUND)))
}
