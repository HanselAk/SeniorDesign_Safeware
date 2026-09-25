const $ = (id) => document.getElementById(id);
let cameraRefreshStarted = false;

function refreshCamera() {
  const image = $("camera-image");
  image.onload = () => setTimeout(refreshCamera, 100);
  image.onerror = () => setTimeout(refreshCamera, 500);
  image.src = `/api/camera/frame?t=${Date.now()}`;
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function chartPoints(values, min, max) {
  if (!values.length) return "";
  return values.map((value, index) => {
    const x = values.length === 1 ? 0 : (index / (values.length - 1)) * 300;
    const y = 60 - ((clamp(value, min, max) - min) / (max - min)) * 54;
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(" ");
}

function setPpeStatus(element, status) {
  element.textContent = status;
  element.classList.toggle("off", status === "MISSING");
  element.classList.toggle("unknown", status === "UNKNOWN");
}

function renderAlerts(alerts) {
  $("alert-count").textContent = `${alerts.length} EVENT${alerts.length === 1 ? "" : "S"}`;
  if (!alerts.length) {
    $("alert-list").innerHTML = '<div class="empty-state">No alerts recorded during this session.</div>';
    return;
  }
  $("alert-list").innerHTML = alerts.map((alert) => `
    <div class="alert-item ${alert.severity === "warning" ? "warning" : ""}">
      <span class="alert-type">${alert.type}</span>
      <span class="alert-message">${alert.message}</span>
      <span class="alert-meta">${alert.time}<br>${alert.lat}, ${alert.lon}</span>
    </div>`).join("");
}

function renderStatus(data) {
  const hero = $("hero-status");
  const status = data.overall_status.toLowerCase();
  hero.className = `hero-status ${status}`;
  $("overall-status").textContent = data.overall_status;
  $("status-symbol").textContent = status === "safe" ? "✓" : status === "emergency" ? "×" : "!";

  if (status === "safe") $("status-summary").textContent = "Camera detected helmet and vest. Check individual sensor labels for live or demo data.";
  if (status === "warning") $("status-summary").textContent = "Camera flagged missing PPE. Check the video and individual sensor labels.";
  if (status === "emergency") $("status-summary").textContent = "An emergency alert is active. Check recent alerts to see its source.";
  if (status === "unverified") $("status-summary").textContent =
    data.motion_has_connected && !data.motion_online ? "Motion sensor disconnected. Check the ESP32 USB cable." :
    "Camera cannot verify PPE right now. Check the camera connection.";
}

function render(data) {
  renderStatus(data);
  $("connection-dot").classList.toggle("online", data.camera_online);
  $("connection-text").textContent = data.camera_online ? "Live camera · demo sensors" : "Camera offline · demo sensors";

  $("hr-value").textContent = data.hr_bpm;
  $("hr-marker").style.left = `${clamp((data.hr_bpm - 40) / 120 * 100, 0, 100)}%`;
  $("hr-caption").textContent = data.hr_alert ? "High heart-rate alert active" : "Normal range";
  $("hr-caption").style.color = data.hr_alert ? "var(--red)" : "var(--muted)";
  $("hr-chart").querySelector("polyline").setAttribute("points", chartPoints(data.hr_history, 55, 140));

  $("temp-value").textContent = Number(data.temp_f).toFixed(1);
  $("temp-c").textContent = `${Number(data.temp_c).toFixed(1)} °C`;
  $("temp-marker").style.left = `${clamp((data.temp_f - 90) / 20 * 100, 0, 100)}%`;
  $("temp-chart").querySelector("polyline").setAttribute("points", chartPoints(data.temp_history, 96, 106));

  setPpeStatus($("helmet-status"), data.helmet_status);
  setPpeStatus($("vest-status"), data.vest_status);
  setPpeStatus($("ai-status"), data.helmet_status === "DETECTED" && data.vest_status === "DETECTED" ? "DETECTED" :
    data.helmet_status === "MISSING" || data.vest_status === "MISSING" ? "MISSING" : "UNKNOWN");

  $("lat").textContent = data.lat;
  $("lon").textContent = data.lon;
  $("alt").textContent = `${data.alt} m`;
  $("smv").textContent = Number(data.smv).toFixed(2);
  $("motion-source").textContent = !data.motion_has_connected ? "DEMO DATA" :
    data.motion_online ? "LIVE MPU6050" : "SENSOR OFFLINE";
  $("ax").textContent = Number(data.ax).toFixed(3);
  $("ay").textContent = Number(data.ay).toFixed(3);
  $("az").textContent = Number(data.az).toFixed(3);
  $("fall-caption").textContent = data.fall_active ? "FALL ALERT ACTIVE" :
    data.motion_has_connected && !data.motion_online ? "Sensor disconnected" : "No fall detected";
  $("fall-caption").style.color = data.fall_active ? "var(--red)" : "var(--muted)";

  $("fps").textContent = data.fps;
  $("detections").replaceChildren(...data.detections.map((item) => {
    const tag = document.createElement("span");
    tag.textContent = item;
    return tag;
  }));
  $("camera-image").hidden = !data.camera_online;
  $("camera-placeholder").hidden = data.camera_online;
  if (data.camera_online && !cameraRefreshStarted) {
    cameraRefreshStarted = true;
    refreshCamera();
  }
  renderAlerts(data.alerts);
}

async function poll() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error("State request failed");
    render(await response.json());
  } catch (error) {
    $("connection-dot").classList.remove("online");
    $("connection-text").textContent = "Dashboard offline";
  }
}

async function trigger(type) {
  await fetch("/api/demo/trigger", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ type }),
  });
  await poll();
}

document.querySelectorAll("[data-trigger]").forEach((button) => {
  button.addEventListener("click", () => trigger(button.dataset.trigger));
});

$("ack-all").addEventListener("click", async () => {
  await fetch("/api/acknowledge/all", { method: "POST" });
  await poll();
});

function updateClock() {
  $("clock").textContent = new Date().toLocaleTimeString([], { hour12: false });
}

updateClock();
setInterval(updateClock, 1000);
poll();
setInterval(poll, 750);
