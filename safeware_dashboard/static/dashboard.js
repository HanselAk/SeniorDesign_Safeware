const $ = (id) => document.getElementById(id);
let cameraStarted = false;

function refreshCamera() {
  const img = $("camera-image");
  img.onload = () => setTimeout(refreshCamera, 100);
  img.onerror = () => setTimeout(refreshCamera, 500);
  img.src = `/api/camera/frame?t=${Date.now()}`;
}

// Colors a tile: "ok" (green), "bad" (red) or "" (gray / unknown).
function setTile(id, state) {
  const tile = $(id);
  tile.classList.remove("ok", "bad");
  if (state) tile.classList.add(state);
}

function gearText(el, status) {
  el.textContent = status === "DETECTED" ? "Detected" : status === "MISSING" ? "Missing" : "Unknown";
  el.className = status === "DETECTED" ? "yes" : status === "MISSING" ? "no" : "unknown";
  const row = el.id === "helmet" ? $("helmet-row") : $("vest-row");
  row.setAttribute("data-state", ["DETECTED", "MISSING"].includes(status) ? status : "UNKNOWN");
}

const STATUS_TEXT = {
  safe: "Camera detected a hard hat and vest. Check each sensor’s connection below.",
  warning: "Camera flagged missing protective equipment. Check the worker and camera view.",
  unverified: "Safety gear could not be confirmed. Check the camera view.",
};

function detailNumber(value, digits = 1, unit = "") {
  return typeof value === "number" && Number.isFinite(value)
    ? `${value.toFixed(digits)}${unit ? " " + unit : ""}` : "--";
}

function renderHistory(prefix, history, unit, digits) {
  const values = Array.isArray(history)
    ? history.filter(v => typeof v === "number" && Number.isFinite(v)).slice(-60) : [];
  const low = values.length ? Math.min(...values) : null;
  const high = values.length ? Math.max(...values) : null;
  const average = values.length ? values.reduce((a, b) => a + b, 0) / values.length : null;
  $(prefix + "-min").textContent = detailNumber(low, digits, unit);
  $(prefix + "-avg").textContent = detailNumber(average, digits, unit);
  $(prefix + "-max").textContent = detailNumber(high, digits, unit);
  $(prefix + "-samples").textContent = values.length
    ? `${values.length} recent simulated readings · oldest → newest` : "History unavailable";
  const span = Math.max((high ?? 0) - (low ?? 0), 1);
  const points = values.map((v, i) => {
    const x = values.length === 1 ? 120 : 4 + i / (values.length - 1) * 232;
    const y = 56 - ((v - ((high + low) / 2 - span / 2)) / span) * 48;
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(" ");
  $(prefix + "-trend").setAttribute("points", points);
}

function renderAdvanced(data) {
  renderHistory("hr", data?.hr_history, "BPM", 0);
  renderHistory("temp", data?.temp_history, "°F", 1);
  $("hr-alert-detail").textContent = !data ? "Unavailable" : data.hr_alert ? "Simulated alert active" : "No demo alert";
  $("temp-alert-detail").textContent = !data ? "Unavailable" : data.temp_alert ? "Simulated alert active" : "No demo alert";
  $("temp-c-detail").textContent = detailNumber(data?.temp_c, 1, "°C");
  const live = data?.motion_has_connected === true && data?.motion_online === true;
  $("fall-source-detail").textContent = live ? "Live wearable" : !data ? "Dashboard disconnected"
    : data.motion_has_connected ? "Wearable disconnected" : "Waiting for wearable";
  $("fall-smv-detail").textContent = detailNumber(live ? data.smv : null, 2, "m/s²");
  for (const [axis, key] of [["x", "ax"], ["y", "ay"], ["z", "az"]]) {
    $("fall-" + axis + "-detail").textContent = detailNumber(live ? data[key] : null, 2, "m/s²");
  }
  $("fall-steps-detail").textContent = detailNumber(live ? data.pdr?.steps : null, 0);
  $("fall-distance-detail").textContent = detailNumber(live ? data.pdr?.distance : null, 1, "m");
}

function render(data) {
  renderAdvanced(data);
  // Big status bar
  const status = data.overall_status.toLowerCase();
  $("status").className = `status ${status}`;
  $("status-word").textContent = {safe: "GEAR DETECTED", warning: "CHECK WORKER",
    emergency: "ALERT ACTIVE", unverified: "NOT CONFIRMED"}[status] || "CHECKING";
  let text = STATUS_TEXT[status] || "";
  if (status === "emergency") text = (data.alerts[0] && data.alerts[0].message) || "Emergency alert active.";
  if (status === "unverified" && !data.camera_online) text = "Camera is offline, so safety gear can't be checked.";
  if (status === "unverified" && data.motion_has_connected && !data.motion_online) text = "The wearable sensor stopped responding.";
  $("status-text").textContent = text;

  $("connection-text").textContent = data.camera_online ? "● Camera connected" : "● Camera offline";

  // Heart rate
  $("hr-value").textContent = data.hr_bpm;
  $("hr-state").textContent = data.hr_alert ? "Simulated high heart rate" : "Simulated reading — not live";
  setTile("tile-hr", data.hr_alert ? "bad" : "");

  // Temperature
  $("temp-value").textContent = Number(data.temp_f).toFixed(1);
  $("temp-state").textContent = data.temp_alert ? "Simulated high temperature" : "Simulated reading — not live";
  setTile("tile-temp", data.temp_alert ? "bad" : "ok");

  // Safety gear
  gearText($("helmet"), data.helmet_status);
  gearText($("vest"), data.vest_status);
  const allOn = data.helmet_status === "DETECTED" && data.vest_status === "DETECTED";
  const missing = data.helmet_status === "MISSING" || data.vest_status === "MISSING";
  $("ppe-state").textContent = allOn ? "Hard hat and vest detected" : missing ? "Check protective equipment" : data.camera_online ? "Camera cannot confirm gear" : "Camera offline";
  setTile("tile-ppe", allOn ? "ok" : missing ? "bad" : "");

  // Fall detection
  const sensorLost = data.motion_has_connected && !data.motion_online;
  const sensorLive = data.motion_has_connected && data.motion_online;
  $("motion-source").textContent = sensorLive ? "(live)" : "";
  $("fall-value").textContent = data.fall_active ? "ALERT" : sensorLive ? "No fall" : "Offline";
  $("fall-state").textContent = data.fall_active ? "Fall alert active — check worker"
    : sensorLost ? "Wearable disconnected — check power / Wi-Fi"
    : sensorLive ? "Wearable connected" : "Waiting for wearable — monitoring unavailable";
  setTile("tile-fall", data.fall_active ? "bad" : sensorLive ? "ok" : "");

  // Battery
  const b = data.battery || {};
  const batteryOn = b.online === true && Number.isFinite(b.percent);
  $("battery-value").textContent = batteryOn ? `~${Math.round(b.percent)}%` : "--";
  $("battery-state").textContent = !batteryOn ? "Battery reading unavailable" : b.low ? "Low battery — recharge" : "Battery connected";
  setTile("tile-battery", !batteryOn ? "" : b.low ? "bad" : "ok");

  // Battery graphic follows the live percentage and clears when unavailable.
  $("battery-fill").setAttribute("width", batteryOn ?
    (Math.max(0, Math.min(100, b.percent)) / 100 * 30).toFixed(1) : "0");

  // Camera
  $("camera-image").hidden = !data.camera_online;
  $("camera-off").hidden = data.camera_online;
  const people = [...new Set((data.workers || []).map((w) => w.label))];
  $("camera-workers").textContent = !data.camera_online ? "--" : people.length ? people.join(", ") : "None";
  if (data.camera_online && !cameraStarted) { cameraStarted = true; refreshCamera(); }

  // Alerts
  $("alerts-count").textContent = String(data.alerts.length);
  const list = $("alerts");
  list.replaceChildren();
  if (!data.alerts.length) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "No alerts yet.";
    list.append(li);
  }
  for (const a of data.alerts) {
    const li = document.createElement("li");
    if (a.severity === "warning") li.className = "warning";
    const msg = document.createElement("span");
    msg.textContent = a.message;
    const time = document.createElement("time");
    time.textContent = a.time;
    li.append(msg, time);
    list.append(li);
  }
}

async function poll() {
  try {
    const res = await fetch("/api/state", { cache: "no-store" });
    if (!res.ok) throw new Error();
    render(await res.json());
  } catch {
    renderAdvanced(null);
    $("connection-text").textContent = "● Dashboard offline";
    $("status").className = "status unverified";
    $("status-word").textContent = "DISCONNECTED";
    $("status-text").textContent = "Live readings unavailable. Check the Jetson connection.";
    for (const id of ["tile-hr", "tile-temp", "tile-ppe", "tile-fall", "tile-battery"]) setTile(id, "");
    for (const id of ["hr-value", "temp-value", "battery-value"]) $(id).textContent = "--";
    for (const id of ["hr-state", "temp-state", "ppe-state", "fall-state", "battery-state"]) $(id).textContent = "Live readings unavailable";
    gearText($("helmet"), "UNKNOWN");
    gearText($("vest"), "UNKNOWN");
    $("battery-fill").setAttribute("width", "0");
    $("fall-value").textContent = "Offline";
    $("motion-source").textContent = "";
    $("camera-image").hidden = true;
    $("camera-off").hidden = false;
    $("camera-workers").textContent = "--";
  }
}

document.querySelectorAll("[data-trigger]").forEach((btn) => {
  btn.addEventListener("click", async () => {
    await fetch("/api/demo/trigger", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ type: btn.dataset.trigger }),
    });
    poll();
  });
});

$("ack-all").addEventListener("click", async () => {
  await fetch("/api/acknowledge/all", { method: "POST" });
  poll();
});

setInterval(() => { $("clock").textContent = new Date().toLocaleTimeString([], { hour: "numeric", minute: "2-digit" }); }, 1000);
poll();
setInterval(poll, 750);
