//! HTTP + WebSocket control API (axum).
//!
//! Per-connection shape:
//!   - Inbound task: parse JSON → `AnyMessage`, dispatch via `RobotCore::dispatch()`,
//!     push per-client responses onto the outbound channel.
//!   - Outbound task: select over (per-connection responses) +
//!     (broadcast sensor stream), forward both to the WS sink.
//!
//! Per-client responses are pre-serialized JSON strings (from dispatch).
//! Broadcast messages (sensor data, daemon status, task progress) go
//! through the broadcast channel and are serialized here.

use std::sync::Arc;

use axum::Router;
use axum::extract::State;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::IntoResponse;
use axum::routing::get;
use futures_util::{SinkExt, StreamExt};
use tokio::sync::{broadcast, mpsc};
use tracing::warn;

use crate::protocol::AnyMessage;
use crate::robot::RobotCore;

/// Capacity of the per-connection outbound mpsc. Keeps a bounded backlog
/// of responses in case a pathological client sends faster than it reads.
const PER_CONN_OUTBOUND_CAPACITY: usize = 64;

#[derive(Clone)]
pub struct AppState {
    pub core: Arc<RobotCore>,
}

pub fn build_router(core: Arc<RobotCore>) -> Router {
    Router::new()
        .route("/ws/sdk", get(ws_handler))
        .route("/ws", get(ws_handler))
        .with_state(AppState { core })
}

async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state.core))
}

async fn handle_socket(socket: WebSocket, core: Arc<RobotCore>) {
    let (mut sink, mut stream) = socket.split();
    let sensor_rx = core.subscribe();
    let (out_tx, out_rx) = mpsc::channel::<String>(PER_CONN_OUTBOUND_CAPACITY);

    // Outbound: fan sensor broadcasts + per-conn replies to the WS sink.
    let outbound = tokio::spawn(async move {
        let mut out_rx = out_rx;
        let mut sensor_rx = sensor_rx;
        loop {
            let json: String = tokio::select! {
                // Per-connection replies (pre-serialized JSON from dispatch).
                m = out_rx.recv() => match m {
                    Some(s) => s,
                    None => break,  // inbound closed
                },
                // Broadcast stream (sensor data, status, task progress).
                m = sensor_rx.recv() => match m {
                    Ok(msg) => match serde_json::to_string(&msg) {
                        Ok(s) => s,
                        Err(e) => {
                            warn!("serialize broadcast: {e}");
                            continue;
                        }
                    },
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => break,
                },
            };
            if sink.send(Message::Text(json.into())).await.is_err() {
                break;
            }
        }
    });

    // Inbound: parse client messages, dispatch, push replies back.
    while let Some(msg) = stream.next().await {
        let Ok(msg) = msg else { break };
        let text = match msg {
            Message::Text(t) => t,
            Message::Binary(_) => continue,
            Message::Close(_) => break,
            Message::Ping(_) | Message::Pong(_) => continue,
        };
        let any_msg: AnyMessage = match serde_json::from_str(&text) {
            Ok(m) => m,
            Err(e) => {
                let err_json = serde_json::json!({
                    "type": "error",
                    "message": format!("bad command: {e}")
                })
                .to_string();
                let _ = out_tx.send(err_json).await;
                continue;
            }
        };
        let replies = core.dispatch(any_msg).await;
        for reply in replies {
            if out_tx.send(reply).await.is_err() {
                break;
            }
        }
    }

    // Dropping out_tx closes the mpsc, which lets the outbound task exit.
    drop(out_tx);
    let _ = outbound.await;
}
