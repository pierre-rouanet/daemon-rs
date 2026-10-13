//! Central signaling relay for WebRTC.
//!
//! Bridges the local GStreamer webrtcsink signaling server (WebSocket at
//! `ws://127.0.0.1:8443`) with the HuggingFace central signaling relay
//! (SSE + HTTP POST at `https://cduss-reachy-mini-central.hf.space`).
//!
//! Remote clients connect via the central relay; this module translates
//! session IDs between the two signaling domains and relays SDP/ICE
//! messages in both directions.

use std::collections::{HashMap, VecDeque};
use std::time::Duration;

use anyhow::{bail, Context, Result};
use futures_util::{SinkExt, StreamExt};
use tokio_tungstenite::tungstenite::Message as WsMessage;
use tracing::{error, info, warn};

const CENTRAL_SERVER: &str = "https://cduss-reachy-mini-central.hf.space";
const LOCAL_WS: &str = "ws://127.0.0.1:8443";
const RECONNECT_SECS: u64 = 5;
const SSE_TIMEOUT_SECS: u64 = 90;

type WsSink = futures_util::stream::SplitSink<
    tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    WsMessage,
>;

type WsStream = futures_util::stream::SplitStream<
    tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
>;

/// Spawn the relay as a background tokio task. Reconnects forever on failure.
pub fn spawn(hf_token: String, robot_name: String) {
    tokio::spawn(async move {
        info!(robot_name, "central signaling relay started");
        loop {
            match relay_once(&hf_token, &robot_name).await {
                Ok(()) => info!("relay connection ended"),
                Err(e) => warn!("relay error: {e:#}"),
            }
            info!("relay reconnecting in {RECONNECT_SECS}s...");
            tokio::time::sleep(Duration::from_secs(RECONNECT_SECS)).await;
        }
    });
}

async fn relay_once(hf_token: &str, robot_name: &str) -> Result<()> {
    // --- 1. Connect to local webrtcsink signaling WS ---
    let (ws, _) = tokio::time::timeout(
        Duration::from_secs(5),
        tokio_tungstenite::connect_async(LOCAL_WS),
    )
    .await
    .context("timeout connecting to local WS")?
    .context("local WS connect failed")?;

    let (mut ws_tx, mut ws_rx) = ws.split();

    // Wait for local welcome
    let welcome_text = recv_text(&mut ws_rx).await.context("no local welcome")?;
    let welcome: serde_json::Value = serde_json::from_str(&welcome_text)?;
    info!("relay local peer: {}", welcome["peerId"]);

    // Register as listener on local signaling server
    ws_send(
        &mut ws_tx,
        &serde_json::json!({
            "type": "setPeerStatus",
            "roles": ["listener"],
            "meta": {"name": "central-relay"}
        }),
    )
    .await?;

    // --- 2. Connect to central SSE ---
    info!("relay connecting to central SSE...");
    let http = reqwest::Client::new();
    let resp = http
        .get(format!("{CENTRAL_SERVER}/events?token={hf_token}"))
        .header("Authorization", format!("Bearer {hf_token}"))
        .send()
        .await
        .context("central SSE connect")?;

    match resp.status().as_u16() {
        200 => {}
        401 => bail!("auth failed — invalid HF token"),
        s => bail!("central returned HTTP {s}"),
    }

    let mut sse = resp.bytes_stream();
    let mut sse_buf = String::new();

    // --- Session state ---
    let mut producer_id: Option<String> = None;
    let mut pending: VecDeque<String> = VecDeque::new(); // central sessions awaiting local
    let mut c2l: HashMap<String, String> = HashMap::new(); // central → local session ID
    let mut l2c: HashMap<String, String> = HashMap::new(); // local → central session ID

    let send_url = format!("{CENTRAL_SERVER}/send?token={hf_token}");
    let auth = format!("Bearer {hf_token}");

    // SSE stale-connection watchdog
    let sse_timeout = tokio::time::sleep(Duration::from_secs(SSE_TIMEOUT_SECS));
    tokio::pin!(sse_timeout);

    // --- 3. Main relay loop ---
    loop {
        tokio::select! {
            // --- Local WS messages ---
            msg = ws_rx.next() => {
                let msg = msg.context("local WS closed")?.context("local WS error")?;
                let text = match msg {
                    WsMessage::Text(t) => t.to_string(),
                    WsMessage::Close(_) => bail!("local WS closed"),
                    _ => continue,
                };
                let v: serde_json::Value = match serde_json::from_str(&text) {
                    Ok(v) => v,
                    Err(_) => continue,
                };

                match v["type"].as_str().unwrap_or("") {
                    "list" => {
                        if let Some(ps) = v["producers"].as_array() {
                            if let Some(p) = ps.first() {
                                producer_id = p["id"].as_str().map(String::from);
                                // Start a local session for each pending central request
                                for _ in 0..pending.len() {
                                    if let Some(pid) = &producer_id {
                                        ws_send(&mut ws_tx, &serde_json::json!({
                                            "type": "startSession",
                                            "peerId": pid,
                                        })).await?;
                                    }
                                }
                            }
                        }
                    }
                    "peerStatusChanged" => {
                        if let Some(roles) = v["roles"].as_array() {
                            if roles.iter().any(|r| r.as_str() == Some("producer")) {
                                producer_id = v["peerId"].as_str().map(String::from);
                            }
                        }
                    }
                    "sessionStarted" => {
                        if let (Some(local_sid), Some(central_sid)) =
                            (v["sessionId"].as_str(), pending.pop_front())
                        {
                            c2l.insert(central_sid.clone(), local_sid.to_string());
                            l2c.insert(local_sid.to_string(), central_sid.clone());
                            info!("relay session mapped: {central_sid} <-> {local_sid}");
                        }
                    }
                    "peer" => {
                        // SDP/ICE from local → relay to central with translated session ID
                        if let Some(local_sid) = v["sessionId"].as_str() {
                            if let Some(central_sid) = l2c.get(local_sid) {
                                let mut m = serde_json::json!({
                                    "type": "peer",
                                    "sessionId": central_sid,
                                });
                                if let Some(sdp) = v.get("sdp") { m["sdp"] = sdp.clone(); }
                                if let Some(ice) = v.get("ice") { m["ice"] = ice.clone(); }
                                post(&http, &send_url, &auth, &m).await;
                            }
                        }
                    }
                    "endSession" => {
                        if let Some(local_sid) = v["sessionId"].as_str() {
                            if let Some(central_sid) = l2c.remove(local_sid) {
                                c2l.remove(&central_sid);
                                post(&http, &send_url, &auth, &serde_json::json!({
                                    "type": "endSession",
                                    "sessionId": central_sid,
                                })).await;
                                info!("relay session ended (local): {local_sid}");
                            }
                        }
                    }
                    _ => {}
                }
            }

            // --- Central SSE events ---
            chunk = sse.next() => {
                let bytes = chunk.context("SSE ended")?.context("SSE error")?;
                sse_timeout.as_mut().reset(
                    tokio::time::Instant::now() + Duration::from_secs(SSE_TIMEOUT_SECS)
                );
                sse_buf.push_str(&String::from_utf8_lossy(&bytes));

                while let Some(nl) = sse_buf.find('\n') {
                    let line = sse_buf[..nl].trim_end().to_string();
                    sse_buf = sse_buf[nl + 1..].to_string();

                    let data = match line.strip_prefix("data:") {
                        Some(d) => d.trim(),
                        None => continue,
                    };
                    if data.is_empty() { continue; }

                    let v: serde_json::Value = match serde_json::from_str(data) {
                        Ok(v) => v,
                        Err(_) => continue,
                    };

                    match v["type"].as_str().unwrap_or("") {
                        "welcome" => {
                            info!("relay central welcome, peer: {}", v["peerId"]);
                            // Register as producer on the central server
                            post(&http, &send_url, &auth, &serde_json::json!({
                                "type": "setPeerStatus",
                                "roles": ["producer"],
                                "meta": {"name": robot_name},
                            })).await;
                            info!("relay registered as producer '{robot_name}'");
                        }
                        "startSession" => {
                            // Remote client wants to connect
                            if let Some(sid) = v["sessionId"].as_str() {
                                info!("relay remote session request: {sid}");
                                pending.push_back(sid.to_string());
                                // Ask local for producer list to start session
                                ws_send(&mut ws_tx, &serde_json::json!({"type": "list"})).await?;
                            }
                        }
                        "peer" => {
                            // SDP/ICE from central → relay to local with translated session ID
                            if let Some(central_sid) = v["sessionId"].as_str() {
                                if let Some(local_sid) = c2l.get(central_sid) {
                                    let mut m = serde_json::json!({
                                        "type": "peer",
                                        "sessionId": local_sid,
                                    });
                                    if let Some(sdp) = v.get("sdp") { m["sdp"] = sdp.clone(); }
                                    if let Some(ice) = v.get("ice") { m["ice"] = ice.clone(); }
                                    ws_send(&mut ws_tx, &m).await?;
                                }
                            }
                        }
                        "endSession" => {
                            if let Some(central_sid) = v["sessionId"].as_str() {
                                pending.retain(|s| s != central_sid);
                                if let Some(local_sid) = c2l.remove(central_sid) {
                                    l2c.remove(&local_sid);
                                    ws_send(&mut ws_tx, &serde_json::json!({
                                        "type": "endSession",
                                        "sessionId": local_sid,
                                    })).await?;
                                    info!("relay session ended (central): {central_sid}");
                                }
                            }
                        }
                        _ => {} // list, peerStatusChanged — ignore
                    }
                }
            }

            // --- SSE stale-connection watchdog ---
            _ = &mut sse_timeout => {
                bail!("no SSE activity for {SSE_TIMEOUT_SECS}s — connection stale");
            }
        }
    }
}

/// Receive the next text WS message (with timeout).
async fn recv_text(rx: &mut WsStream) -> Result<String> {
    let msg = tokio::time::timeout(Duration::from_secs(3), rx.next())
        .await
        .context("timeout")?
        .context("stream ended")?
        .context("ws error")?;
    match msg {
        WsMessage::Text(t) => Ok(t.to_string()),
        other => bail!("expected text, got {other:?}"),
    }
}

/// Send JSON as text to local WS.
async fn ws_send(tx: &mut WsSink, msg: &serde_json::Value) -> Result<()> {
    tx.send(WsMessage::Text(msg.to_string().into()))
        .await
        .context("send to local WS")
}

/// POST JSON to central signaling server (fire-and-forget).
async fn post(http: &reqwest::Client, url: &str, auth: &str, msg: &serde_json::Value) {
    if let Err(e) = http
        .post(url)
        .header("Authorization", auth)
        .json(msg)
        .send()
        .await
    {
        warn!("relay POST to central failed: {e}");
    }
}
