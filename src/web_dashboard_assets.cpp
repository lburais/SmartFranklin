#include "web_dashboard_assets.h"
#include <Arduino.h>

const char THEME_JS[] PROGMEM = R"JS(
// SPDX-License-Identifier: MIT

// Default theme = dark
if (!localStorage.getItem("theme")) {
    localStorage.setItem("theme", "dark");
}

function applyTheme() {
    const t = localStorage.getItem("theme");

    if (t === "light") {
        document.documentElement.style.setProperty("--bg", "#f5f5f5");
        document.documentElement.style.setProperty("--fg", "#111");
        document.documentElement.style.setProperty("--fg-soft", "#444");
        document.documentElement.style.setProperty("--fg-strong", "#000");
        document.documentElement.style.setProperty("--card", "#fff");
        document.documentElement.style.setProperty("--border", "#ccc");
        document.documentElement.style.setProperty("--accent", "#0077cc");
        document.documentElement.style.setProperty("--accent-glow", "#66bfff");
    } else {
        document.documentElement.style.setProperty("--bg", "#0a0a0f");
        document.documentElement.style.setProperty("--fg", "#e0e0ff");
        document.documentElement.style.setProperty("--fg-soft", "#8aa");
        document.documentElement.style.setProperty("--fg-strong", "#fff");
        document.documentElement.style.setProperty("--card", "#11111a");
        document.documentElement.style.setProperty("--border", "#222");
        document.documentElement.style.setProperty("--accent", "#6ecbff");
        document.documentElement.style.setProperty("--accent-glow", "#0099ff");
    }
}

applyTheme();

// Toggle theme
function toggleTheme() {
    const t = localStorage.getItem("theme");
    localStorage.setItem("theme", t === "light" ? "dark" : "light");
    applyTheme();
}
)JS";

const char HW_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>SmartFranklin - Hardware Monitor</title>
    <script src="/theme.js"></script>
    <style>
        body { font-family: Arial, sans-serif; background: var(--bg); color: var(--fg); margin: 0; padding: 20px; }
        h1 { color: var(--accent); margin-bottom: 10px; text-shadow: 0 0 8px var(--accent-glow); }
        .nav { margin-bottom: 14px; }
        .nav a { display: inline-block; padding: 8px 12px; margin-right: 6px; border-radius: 6px; text-decoration: none; background: var(--accent); color: #fff; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 20px; }
        .card { background: var(--card); padding: 15px; border-radius: 12px; border: 1px solid var(--border); }
        .value { font-size: 1.6em; font-weight: bold; color: var(--fg-strong); }
        .label { font-size: 0.9em; color: var(--fg-soft); }
        .btn { display: inline-block; padding: 10px 14px; background: #1a1a2a; border-radius: 6px; margin-top: 10px; color: #ccc; cursor: pointer; user-select: none; border: 1px solid #333; }
        .btn.active { background: #00c853; color: #fff; box-shadow: 0 0 10px #00ff88; }
        .slider { width: 100%; }
        canvas { width: 100%; height: 180px; background: #000; border-radius: 8px; }
    </style>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
<h1>Hardware Monitor</h1>
<div class="nav">
    <a href="/">Dashboard</a>
    <a href="/config">Configuration</a>
    <a href="/sensors">Sensors</a>
    <a href="/level">Level</a>
</div>

<div class="grid">
    <div class="card"><div class="label">Battery Voltage</div><div id="battery_voltage" class="value">--</div></div>
    <div class="card"><div class="label">Battery Level</div><div id="battery_percent" class="value">-- %</div></div>
    <div class="card"><div class="label">Charging</div><div id="charging" class="value">--</div></div>
    <div class="card"><div class="label">Temperature</div><div id="temperature" class="value">-- C</div></div>
    <div class="card"><div class="label">Button A</div><div id="button_a" class="btn">Released</div></div>
    <div class="card"><div class="label">Button B</div><div id="button_b" class="btn">Released</div></div>
    <div class="card"><div class="label">Brightness</div><input id="brightness" type="range" min="10" max="255" value="128" class="slider"></div>
    <div class="card"><div class="label">Actions</div><div id="reboot" class="btn">Reboot</div><div id="sleep" class="btn">Deep Sleep</div></div>
</div>

<h2 style="color:var(--accent);margin-top:30px;">Live Charts</h2>
<div class="grid">
    <div class="card"><div class="label">Battery Voltage (V)</div><canvas id="chart_batt"></canvas></div>
    <div class="card"><div class="label">IMU Acceleration (XYZ)</div><canvas id="chart_imu"></canvas></div>
</div>

<script>
let battChart, imuChart;

function initCharts() {
    const ctx1 = document.getElementById('chart_batt').getContext('2d');
    battChart = new Chart(ctx1, {
        type: 'line',
        data: { labels: [], datasets: [{ label: "Voltage", data: [], borderColor: "#6ecbff", borderWidth: 2 }] },
        options: { animation: false }
    });

    const ctx2 = document.getElementById('chart_imu').getContext('2d');
    imuChart = new Chart(ctx2, {
        type: 'line',
        data: { labels: [], datasets: [
            { label: "X", data: [], borderColor: "#ff5252", borderWidth: 2 },
            { label: "Y", data: [], borderColor: "#4caf50", borderWidth: 2 },
            { label: "Z", data: [], borderColor: "#ffeb3b", borderWidth: 2 }
        ] },
        options: { animation: false }
    });
}

function updateHw() {
    fetch("/api/hw").then(r => r.json()).then(data => {
        document.getElementById("battery_voltage").textContent = data.battery_voltage.toFixed(2) + " V";
        document.getElementById("battery_percent").textContent = data.battery_percent + " %";
        document.getElementById("charging").textContent = data.charging ? "Charging" : "Not Charging";
        document.getElementById("temperature").textContent = data.temperature.toFixed(1) + " C";

        const btnA = document.getElementById("button_a");
        const btnB = document.getElementById("button_b");
        btnA.classList.toggle("active", data.button_a);
        btnA.textContent = data.button_a ? "Pressed" : "Released";
        btnB.classList.toggle("active", data.button_b);
        btnB.textContent = data.button_b ? "Pressed" : "Released";

        const t = new Date().toLocaleTimeString();
        battChart.data.labels.push(t);
        battChart.data.datasets[0].data.push(data.battery_voltage);
        if (battChart.data.labels.length > 20) {
            battChart.data.labels.shift();
            battChart.data.datasets[0].data.shift();
        }
        battChart.update();

        imuChart.data.labels.push(t);
        imuChart.data.datasets[0].data.push(data.accel.x);
        imuChart.data.datasets[1].data.push(data.accel.y);
        imuChart.data.datasets[2].data.push(data.accel.z);
        if (imuChart.data.labels.length > 20) {
            imuChart.data.labels.shift();
            imuChart.data.datasets.forEach(ds => ds.data.shift());
        }
        imuChart.update();
    });
}

document.getElementById("brightness").addEventListener("input", e => fetch("/api/set_brightness?value=" + e.target.value));
document.getElementById("reboot").addEventListener("click", () => fetch("/api/reboot"));
document.getElementById("sleep").addEventListener("click", () => fetch("/api/sleep"));

initCharts();
setInterval(updateHw, 2000);
updateHw();
</script>
</body>
</html>
)HTML";

const char SENSORS_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>SmartFranklin - Sensors Monitor</title>
    <script src="/theme.js"></script>
    <style>
        body { font-family: Arial; background: var(--bg); color: var(--fg); padding:20px; }
        h1 { color: var(--accent); text-shadow:0 0 8px var(--accent-glow); }
        .nav { margin-bottom: 14px; }
        .nav a { display: inline-block; padding: 8px 12px; margin-right: 6px; border-radius: 6px; text-decoration: none; background: var(--accent); color: #fff; }
        .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(260px,1fr)); gap:20px; }
        .card { background:var(--card); padding:15px; border-radius:12px; border:1px solid var(--border); }
        .value { font-size:1.6em; font-weight:bold; color:var(--fg-strong); }
        .label { font-size:0.9em; color:var(--fg-soft); }
        canvas { width:100%; height:180px; background:#000; border-radius:8px; }
    </style>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>

<h1>Sensors Monitor</h1>
<div class="nav">
    <a href="/">Dashboard</a>
    <a href="/config">Configuration</a>
    <a href="/hw">Hardware</a>
    <a href="/level">Level</a>
</div>

<div class="grid">
    <div class="card"><div class="label">Weight (U180)</div><div id="weight" class="value">-- g</div></div>
    <div class="card"><div class="label">Temperature</div><div id="temp" class="value">-- C</div></div>
    <div class="card"><div class="label">IMU Pitch / Roll</div><div id="imu_pose" class="value">-- / -- deg</div></div>
    <div class="card"><div class="label">Wheel Heights (FL FR RL RR)</div><div id="imu_wheels" class="value" style="font-size:1.0em;line-height:1.4;">-- -- -- -- mm</div></div>
    <div class="card">
        <div class="label">IMU Geometry Calibration</div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px;">
            <input id="wheelbase_mm" type="number" step="1" placeholder="Wheelbase mm" />
            <input id="track_width_mm" type="number" step="1" placeholder="Track width mm" />
            <input id="offset_x_mm" type="number" step="1" placeholder="Offset X mm" />
            <input id="offset_y_mm" type="number" step="1" placeholder="Offset Y mm" />
        </div>
        <div style="margin-top:10px;display:flex;gap:8px;">
            <button onclick="saveImuGeometry()">Save IMU Geometry</button>
            <button onclick="loadImuGeometry()">Reload</button>
        </div>
        <div id="imu_geom_status" class="label" style="margin-top:8px;">--</div>
    </div>
</div>

<h2 style="color:var(--accent);margin-top:30px;">Live Charts</h2>
<div class="grid">
    <div class="card"><div class="label">Weight (g)</div><canvas id="chart_weight"></canvas></div>
</div>

<script>
let weightChart;

function initCharts() {
    weightChart = new Chart(document.getElementById('chart_weight'), {
        type: 'line',
        data: { labels: [], datasets: [{ label:"Weight", data:[], borderColor:"#6ecbff", borderWidth:2 }] },
        options: { animation:false }
    });
}

function update() {
    Promise.all([
        fetch("/api/status").then(r => r.json()),
        fetch("/api/hw").then(r => r.json())
    ]).then(([status, hw]) => {
        const weight = Number(status.weight_gaz || 0);
        const temp = Number(hw.temperature || 0);

        document.getElementById("weight").textContent = weight + " g";
        document.getElementById("temp").textContent = temp.toFixed(1) + " C";

        const pitch = Number(status.imu_pitch_deg || 0);
        const roll = Number(status.imu_roll_deg || 0);
        document.getElementById("imu_pose").textContent = pitch.toFixed(2) + " / " + roll.toFixed(2) + " deg";

        const fl = Number(status.imu_wheel_fl_mm || 0);
        const fr = Number(status.imu_wheel_fr_mm || 0);
        const rl = Number(status.imu_wheel_rl_mm || 0);
        const rr = Number(status.imu_wheel_rr_mm || 0);
        document.getElementById("imu_wheels").textContent =
            "FL " + fl.toFixed(1) + "  FR " + fr.toFixed(1) + "\n" +
            "RL " + rl.toFixed(1) + "  RR " + rr.toFixed(1) + " mm";

        const t = new Date().toLocaleTimeString();
        weightChart.data.labels.push(t);
        weightChart.data.datasets[0].data.push(weight);
        if (weightChart.data.labels.length > 20) {
            weightChart.data.labels.shift();
            weightChart.data.datasets[0].data.shift();
        }
        weightChart.update();
    }).catch(() => {
        document.getElementById("imu_geom_status").textContent = "Live update failed";
    });
}

function loadImuGeometry() {
    fetch("/api/imu_geometry")
        .then(r => r.json())
        .then(g => {
            document.getElementById("wheelbase_mm").value = g.imu_wheelbase_mm;
            document.getElementById("track_width_mm").value = g.imu_track_width_mm;
            document.getElementById("offset_x_mm").value = g.imu_offset_x_mm;
            document.getElementById("offset_y_mm").value = g.imu_offset_y_mm;
            document.getElementById("imu_geom_status").textContent = "Geometry loaded";
        })
        .catch(() => {
            document.getElementById("imu_geom_status").textContent = "Failed to load geometry";
        });
}

function saveImuGeometry() {
    const wheelbase = encodeURIComponent(document.getElementById("wheelbase_mm").value);
    const track = encodeURIComponent(document.getElementById("track_width_mm").value);
    const ox = encodeURIComponent(document.getElementById("offset_x_mm").value);
    const oy = encodeURIComponent(document.getElementById("offset_y_mm").value);

    const url = "/api/set_imu_geometry?wheelbase_mm=" + wheelbase +
                "&track_width_mm=" + track +
                "&offset_x_mm=" + ox +
                "&offset_y_mm=" + oy;

    fetch(url)
        .then(r => r.json().then(j => ({ ok: r.ok, body: j })))
        .then(({ ok, body }) => {
            if (!ok) {
                document.getElementById("imu_geom_status").textContent = "Save failed: " + (body.error || "unknown");
                return;
            }

            document.getElementById("wheelbase_mm").value = body.imu_wheelbase_mm;
            document.getElementById("track_width_mm").value = body.imu_track_width_mm;
            document.getElementById("offset_x_mm").value = body.imu_offset_x_mm;
            document.getElementById("offset_y_mm").value = body.imu_offset_y_mm;
            document.getElementById("imu_geom_status").textContent = body.saved ? "Saved and applied" : "Applied but not persisted";
        })
        .catch(() => {
            document.getElementById("imu_geom_status").textContent = "Save request failed";
        });
}

initCharts();
loadImuGeometry();
setInterval(update, 2000);
update();
</script>

</body>
</html>
)HTML";

/**
 * @brief Main dashboard HTML page served from PROGMEM.
 * 
 * Complete HTML document with embedded CSS and JavaScript for the main dashboard.
 * Includes navigation buttons, live data display area, and auto-refresh functionality.
 * Stored in program memory to save RAM (PROGMEM attribute).
 * 
 * Page Features:
 *   - Responsive design with card-based layout
 *   - Navigation buttons to configuration sections
 *   - Live data display in &lt;pre&gt; element
 *   - JavaScript fetch API for real-time updates
 *   - 1-second polling interval for data refresh
 *   - JSON formatting with syntax highlighting
 * 
 * JavaScript Functionality:
 *   - refresh() function fetches /api/status and updates display
 *   - setInterval() calls refresh every 1000ms
 *   - Error handling for network failures
 *   - Automatic retry on fetch failures
 * 
 * CSS Styling:
 *   - Sans-serif font for readability
 *   - Card layout with rounded corners and padding
 *   - Blue button styling for navigation
 *   - Pre-formatted text for JSON display
 */
const char MAIN_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Data</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
pre { white-space: pre-wrap; word-wrap: break-word; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
</style>
</head>
<body>
<h1>SmartFranklin Data</h1>
<div class="card">
    <a class="button" href="/logs">Logs</a>
</div>
<div class="card">
  <h3>Live Data</h3>
  <pre id="data">Loading...</pre>
</div>
<script>
async function refresh() {
  try {
    const r = await fetch('/api/data');
    const j = await r.json();
    document.getElementById('data').textContent = JSON.stringify(j, null, 2);
  } catch (e) {
    document.getElementById('data').textContent = 'Error loading data...';
  }
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
)HTML";

const char CONFIG_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Config</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
pre { white-space: pre-wrap; word-wrap: break-word; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
.grid2 { display: grid; grid-template-columns: repeat(2, minmax(160px, 1fr)); gap: 10px; }
.field { display: flex; flex-direction: column; }
label { font-size: 0.9em; margin-bottom: 4px; }
input { padding: 8px; border: 1px solid #bbb; border-radius: 6px; width: 100%; box-sizing: border-box; }
.actions { margin-top: 12px; display: flex; gap: 8px; }
.btn { padding: 9px 12px; border: 0; border-radius: 6px; background: #0078ff; color: #fff; cursor: pointer; }
.btn.secondary { background: #666; }
.status { color: #333; font-size: 0.9em; margin-top: 10px; }
</style>
</head>
<body>
<h1>Configuration</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/level">Level</a>
    <a class="button" href="/gaz">Gaz 907</a>
    <a class="button" href="/diagnostics">Diagnostics</a>
    <a class="button" href="/update">Firmware Update</a>
    <a class="button" href="/logs">Logs</a>
</div>
<div class="card">
    <h3>Edit Configuration</h3>
    <div id="fields" class="grid2"></div>
    <div class="actions">
        <button class="btn" onclick="saveConfig()">Save</button>
        <button class="btn secondary" onclick="loadConfig()">Reload</button>
    </div>
    <div id="status" class="status">Loading...</div>
</div>
<div class="card">
    <h3>Current Config JSON</h3>
    <pre id="config">Loading...</pre>
</div>
<script>
const fields = [
    { key: 'hostname', label: 'Hostname', type: 'text' },
    { key: 'ap_ssid', label: 'AP SSID', type: 'text' },
    { key: 'ap_pass', label: 'AP Password', type: 'password' },
    { key: 'sta_ssid', label: 'STA SSID', type: 'text' },
    { key: 'sta_pass', label: 'STA Password', type: 'password' },
    { key: 'admin_user', label: 'Admin User', type: 'text' },
    { key: 'admin_pass', label: 'Admin Password', type: 'password' },
    { key: 'mqtt_port', label: 'MQTT Port', type: 'number', step: '1' },
    { key: 'gaz_calibration_factor', label: 'Gaz Calibration Factor', type: 'number', step: '0.01' },
    { key: 'gaz_weight_average_window', label: 'Gaz Average Window', type: 'number', step: '1' },
    { key: 'level_wheelbase_mm', label: 'Level Wheelbase mm', type: 'number', step: '1' },
    { key: 'level_track_width_mm', label: 'Level Track Width mm', type: 'number', step: '1' },
    { key: 'level_offset_x_mm', label: 'Level Offset X mm', type: 'number', step: '1' },
    { key: 'level_offset_y_mm', label: 'Level Offset Y mm', type: 'number', step: '1' },
    { key: 'level_zero_pitch_deg', label: 'Level Zero Pitch deg', type: 'number', step: '0.01' },
    { key: 'level_zero_roll_deg', label: 'Level Zero Roll deg', type: 'number', step: '0.01' },
    { key: 'rtc_timezone', label: 'RTC Timezone', type: 'text' },
    { key: 'task_wifi_loop_ms', label: 'Task WiFi Loop ms', type: 'number', step: '1' },
    { key: 'task_mqtt_loop_ms', label: 'Task MQTT Loop ms', type: 'number', step: '1' },
    { key: 'task_i2c_loop_ms', label: 'Task I2C Loop ms', type: 'number', step: '1' },
    { key: 'task_hmi_loop_ms', label: 'Task HMI Loop ms', type: 'number', step: '1' },
    { key: 'task_hmi_init_retry_ms', label: 'Task HMI Init Retry ms', type: 'number', step: '1' },
    { key: 'task_hw_monitor_loop_ms', label: 'Task HW Monitor Loop ms', type: 'number', step: '1' },
    { key: 'task_bms_ble_connected_loop_ms', label: 'Task BMS Connected Loop ms', type: 'number', step: '1' },
    { key: 'task_bms_ble_retry_loop_ms', label: 'Task BMS Retry Loop ms', type: 'number', step: '1' },
    { key: 'task_watchdog_loop_ms', label: 'Task Watchdog Loop ms', type: 'number', step: '1' }
];

function renderFields() {
    const container = document.getElementById('fields');
    container.innerHTML = '';

    fields.forEach((f) => {
        const wrapper = document.createElement('div');
        wrapper.className = 'field';

        const label = document.createElement('label');
        label.htmlFor = f.key;
        label.textContent = f.label;

        const input = document.createElement('input');
        input.id = f.key;
        input.type = f.type || 'text';
        if (f.step) {
            input.step = f.step;
        }

        wrapper.appendChild(label);
        wrapper.appendChild(input);
        container.appendChild(wrapper);
    });
}

async function loadConfig() {
    document.getElementById('status').textContent = 'Loading...';
    try {
        const r = await fetch('/api/config');
        const j = await r.json();

        fields.forEach((f) => {
            const value = j[f.key];
            document.getElementById(f.key).value = (value === undefined || value === null) ? '' : String(value);
        });

        document.getElementById('config').textContent = JSON.stringify(j, null, 2);
        document.getElementById('status').textContent = 'Configuration loaded';
    } catch (e) {
        document.getElementById('status').textContent = 'Error loading configuration';
        document.getElementById('config').textContent = 'Error loading config';
    }
}

async function saveConfig() {
    const params = new URLSearchParams();
    fields.forEach((f) => {
        params.set(f.key, document.getElementById(f.key).value);
    });

    document.getElementById('status').textContent = 'Saving...';

    try {
        const r = await fetch('/api/set_config?' + params.toString());
        const body = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Save failed: ' + (body.parameter || body.error || 'unknown');
            return;
        }

        const cfg = body.config || {};
        document.getElementById('config').textContent = JSON.stringify(cfg, null, 2);
        document.getElementById('status').textContent = body.saved ? 'Saved and applied' : 'Applied but not persisted';
    } catch (e) {
        document.getElementById('status').textContent = 'Save request failed';
    }
}

renderFields();
loadConfig();
</script>
</body>
</html>
)HTML";

const char GAZ_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Gaz 907</title>
<style>
:root {
    --bg: #f6f7fb;
    --card: #ffffff;
    --ink: #1f2d3d;
    --accent: #0d9488;
    --accent-soft: #99f6e4;
    --line: #cbd5e1;
}
body { background: radial-gradient(circle at 10% 10%, #ecfeff, #f8fafc 45%, #eef2ff); color: var(--ink); font-family: sans-serif; margin: 10px; }
.card { background: var(--card); border: 1px solid var(--line); padding: 15px; border-radius: 12px; margin-bottom: 15px; }
a.button { display: inline-block; padding: 10px 15px; background: #0f766e; color: white; border-radius: 8px; text-decoration: none; margin-right: 8px; }
.layout { display: grid; grid-template-columns: 1fr; gap: 16px; }
.gaz-stage { display: flex; flex-direction: column; align-items: center; gap: 8px; }
.bottle-wrap { width: 210px; height: 360px; position: relative; }
.bottle-neck { position: absolute; top: 0; left: 78px; width: 54px; height: 54px; border: 5px solid #475569; border-bottom: 0; border-radius: 10px 10px 0 0; background: #e2e8f0; }
.bottle-body { position: absolute; left: 20px; top: 40px; width: 170px; height: 300px; border: 5px solid #475569; border-radius: 76px 76px 64px 64px; background: linear-gradient(180deg, #f8fafc 0%, #e2e8f0 100%); overflow: hidden; }
.bottle-fill { position: absolute; left: 0; right: 0; bottom: 0; height: 0%; background: linear-gradient(180deg, #34d399 0%, #0f766e 100%); transition: height 0.5s ease; }
.bottle-highlight { position: absolute; left: 34px; top: 58px; width: 20px; height: 240px; border-radius: 14px; background: rgba(255,255,255,0.42); }
.overlay { position: absolute; left: 0; right: 0; top: 48%; transform: translateY(-50%); text-align: center; font-size: 2.3rem; font-weight: 700; color: #0b3a36; text-shadow: 0 1px 0 rgba(255,255,255,0.55); }
.legend { font-size: 0.95rem; color: #334155; }
.weight { font-size: 1.25rem; font-weight: 700; }
.grid2 { display: grid; grid-template-columns: repeat(2, minmax(150px, 1fr)); gap: 10px; }
label { font-size: 0.9em; margin-bottom: 4px; display: block; }
input { width: 100%; box-sizing: border-box; padding: 9px; border: 1px solid #94a3b8; border-radius: 8px; }
.actions { margin-top: 10px; display: flex; gap: 8px; flex-wrap: wrap; }
.btn { padding: 9px 12px; border: 0; border-radius: 8px; color: #fff; background: #0f766e; cursor: pointer; }
.btn.step2 { background: #0369a1; }
.btn.reload { background: #475569; }
.status { margin-top: 10px; color: #334155; font-size: 0.95rem; min-height: 1.2em; }
.debug { margin-top: 10px; padding: 8px; border: 1px dashed #94a3b8; border-radius: 8px; background: #f8fafc; color: #0f172a; font-size: 0.92rem; white-space: pre-wrap; }
@media (max-width: 420px) {
    .bottle-wrap { width: 180px; height: 320px; }
    .bottle-neck { left: 66px; }
    .bottle-body { left: 15px; width: 150px; height: 270px; }
}
</style>
</head>
<body>
<h1>Gaz 907</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/level">Level</a>
    <a class="button" href="/config">Configuration</a>
    <a class="button" href="/diagnostics">Diagnostics</a>
    <a class="button" href="/logs">Logs</a>
</div>

<div class="card layout">
    <div class="gaz-stage">
        <div class="bottle-wrap" aria-label="Bouteille Gaz 907">
            <div class="bottle-neck"></div>
            <div class="bottle-body">
                <div id="bottle_fill" class="bottle-fill"></div>
                <div class="bottle-highlight"></div>
                <div id="fill_overlay" class="overlay">0%</div>
            </div>
        </div>
        <div class="legend">Niveau estime sur plage 907 (3700g vide, 6450g plein)</div>
        <div id="weight_text" class="weight">0 g</div>
    </div>
</div>

<div class="card">
    <h3>Calibration de la balance</h3>
    <p>Etape 1: faire la tare a zero avec la bouteille vide ou retiree.</p>
    <div class="actions">
        <button class="btn" onclick="tareZero()">Etape 1 - Tare a zero</button>
    </div>

    <p style="margin-top:12px;">Etape 2: poser un poids de reference, saisir sa valeur, puis calibrer.</p>
    <div class="grid2">
        <div>
            <label for="known_weight_g">Poids de reference (g)</label>
            <input id="known_weight_g" type="number" step="1" min="1" value="1000" />
        </div>
        <div>
            <label for="current_raw_g">Mesure capteur actuelle (g)</label>
            <input id="current_raw_g" type="number" step="1" readonly />
        </div>
    </div>
    <div class="actions">
        <button class="btn step2" onclick="applyCalibration()">Etape 2 - Calibrer</button>
        <button class="btn reload" onclick="refreshGaz()">Rafraichir</button>
    </div>
    <div id="status" class="status">Chargement...</div>
    <div id="calib_debug" class="debug">Diagnostics calibration: chargement...</div>
</div>

<script>
function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
}

function renderGaz(data) {
    const fill = clamp(Number(data.fill_gaz || 0), 0, 100);
    const weight = Number(data.weight_gaz || 0);
    document.getElementById('bottle_fill').style.height = fill + '%';
    document.getElementById('fill_overlay').textContent = fill + '%';
    document.getElementById('weight_text').textContent = weight + ' g';
    document.getElementById('current_raw_g').value = weight;
}

function renderCalibrationDebug(data) {
    const lines = [];
    lines.push('Diagnostics calibration');
    lines.push('config gaz_calibration_factor: ' + String(data.gaz_calibration_factor_config));
    lines.push('live sensor gap: ' + (data.sensor_gap_read_ok ? String(data.sensor_gap) : 'read_failed'));
    lines.push('live raw adc: ' + (data.raw_adc_read_ok ? String(data.raw_adc) : 'read_failed'));
    lines.push('live sample g: ' + String(data.sample_weight_g));
    document.getElementById('calib_debug').textContent = lines.join('\n');
}

async function refreshGaz() {
    try {
        const r = await fetch('/api/gaz_status');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Erreur lecture Gaz';
            return;
        }
        renderGaz(j);

        const dr = await fetch('/api/gaz_calibration_debug');
        const dj = await dr.json();
        if (dr.ok) {
            renderCalibrationDebug(dj);
        } else {
            document.getElementById('calib_debug').textContent = 'Diagnostics calibration: indisponibles';
        }

        document.getElementById('status').textContent = 'Donnees Gaz a jour';
    } catch (e) {
        document.getElementById('status').textContent = 'Erreur reseau';
        document.getElementById('calib_debug').textContent = 'Diagnostics calibration: erreur reseau';
    }
}

async function tareZero() {
    document.getElementById('status').textContent = 'Tare en cours...';
    try {
        const r = await fetch('/api/gaz_calibration_tare');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Echec tare: ' + (j.error || 'unknown');
            return;
        }
        await refreshGaz();
        document.getElementById('status').textContent = j.saved ? 'Tare faite et config sauvee' : 'Tare faite';
    } catch (e) {
        document.getElementById('status').textContent = 'Echec tare (reseau)';
    }
}

async function applyCalibration() {
    const known = Number(document.getElementById('known_weight_g').value);
    if (!Number.isFinite(known) || known <= 0) {
        document.getElementById('status').textContent = 'Poids de reference invalide';
        return;
    }

    document.getElementById('status').textContent = 'Calibration en cours...';
    try {
        const url = '/api/gaz_calibration_apply?known_weight_g=' + encodeURIComponent(String(known));
        const r = await fetch(url);
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Echec calibration: ' + (j.error || j.parameter || 'unknown');
            return;
        }
        await refreshGaz();
        document.getElementById('status').textContent = 'Calibration appliquee (factor=' + j.gaz_calibration_factor + ')';
    } catch (e) {
        document.getElementById('status').textContent = 'Echec calibration (reseau)';
    }
}

setInterval(refreshGaz, 1500);
refreshGaz();
</script>
</body>
</html>
)HTML";

const char LEVEL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Level</title>
<style>
:root {
    --line: #cbd5e1;
    --panel: #ffffff;
    --ink: #0f172a;
    --accent: #0284c7;
    --accent2: #0f766e;
}
body { margin: 10px; color: var(--ink); font-family: sans-serif; background: radial-gradient(circle at 15% 10%, #e0f2fe, #f8fafc 42%, #eef2ff); }
.card { background: var(--panel); border: 1px solid var(--line); padding: 15px; border-radius: 12px; margin-bottom: 15px; }
a.button { display: inline-block; padding: 10px 15px; background: #0f766e; color: #fff; border-radius: 8px; text-decoration: none; margin-right: 8px; margin-bottom: 6px; }
.scene { display: flex; flex-direction: column; align-items: center; gap: 10px; }
.van-ground { width: 320px; max-width: 94vw; height: 220px; border-radius: 14px; position: relative; background: linear-gradient(180deg, #f8fafc 0%, #dbeafe 100%); overflow: hidden; border: 1px solid #93c5fd; }
.van-wrap { position: absolute; left: 50%; top: 54%; transform-style: preserve-3d; transform: translate(-50%, -50%) perspective(700px) rotateX(0deg) rotateZ(0deg); transition: transform 0.35s ease; }
.van-body { width: 210px; height: 96px; border: 4px solid #334155; border-radius: 18px 28px 14px 14px; background: linear-gradient(180deg, #fef3c7 0%, #fbbf24 78%); position: relative; }
.van-window { position: absolute; top: 10px; left: 128px; width: 54px; height: 28px; border-radius: 8px; border: 2px solid #334155; background: #dbeafe; }
.van-door { position: absolute; top: 16px; left: 84px; width: 40px; height: 72px; border-left: 2px solid #334155; }
.wheel { width: 26px; height: 26px; border-radius: 50%; background: #111827; border: 3px solid #94a3b8; position: absolute; }
.fl { left: 22px; top: 8px; }
.fr { right: 22px; top: 8px; }
.rl { left: 22px; bottom: 8px; }
.rr { right: 22px; bottom: 8px; }
.wheel-label { position: absolute; font-size: 0.88rem; font-weight: 700; color: #1e293b; background: rgba(255,255,255,0.72); border-radius: 6px; padding: 1px 5px; }
.lbl-fl { left: 8px; top: 40px; }
.lbl-fr { right: 8px; top: 40px; }
.lbl-rl { left: 8px; bottom: 40px; }
.lbl-rr { right: 8px; bottom: 40px; }
.gauges { display: grid; grid-template-columns: repeat(2, minmax(140px, 1fr)); gap: 12px; margin-top: 6px; }
.gauge { background: #f8fafc; border: 1px solid #cbd5e1; border-radius: 10px; padding: 8px; text-align: center; }
.gauge svg { width: 100%; height: 88px; }
.g-title { font-weight: 700; margin-bottom: 4px; }
.g-value { margin-top: 2px; font-size: 1.02rem; font-weight: 700; }
.actions { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 8px; }
.btn { border: 0; border-radius: 8px; padding: 9px 12px; cursor: pointer; color: #fff; background: var(--accent2); }
.btn.secondary { background: #475569; }
.status { margin-top: 8px; min-height: 1.2em; color: #334155; font-size: 0.95rem; }
.legend { font-size: 0.92rem; color: #334155; }
</style>
</head>
<body>
<h1>Level</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/config">Configuration</a>
    <a class="button" href="/gaz">Gaz 907</a>
    <a class="button" href="/diagnostics">Diagnostics</a>
    <a class="button" href="/logs">Logs</a>
</div>

<div class="card scene">
    <div class="van-ground">
        <div id="van_wrap" class="van-wrap">
            <div class="van-body">
                <div class="van-window"></div>
                <div class="van-door"></div>
            </div>
        </div>

        <div class="wheel fl"></div>
        <div class="wheel fr"></div>
        <div class="wheel rl"></div>
        <div class="wheel rr"></div>

        <div id="lbl_fl" class="wheel-label lbl-fl">0.0 cm</div>
        <div id="lbl_fr" class="wheel-label lbl-fr">0.0 cm</div>
        <div id="lbl_rl" class="wheel-label lbl-rl">0.0 cm</div>
        <div id="lbl_rr" class="wheel-label lbl-rr">0.0 cm</div>
    </div>
    <div class="legend">Renault Trafic visualise selon pitch/roll. Hauteurs roues en centimetres.</div>
    <div class="gauges">
        <div class="gauge">
            <div class="g-title">Pitch</div>
            <svg viewBox="0 0 120 70" aria-label="Pitch gauge">
                <path d="M10,60 A50,50 0 0 1 110,60" fill="none" stroke="#94a3b8" stroke-width="8"/>
                <line id="pitch_needle" x1="60" y1="60" x2="60" y2="16" stroke="#0284c7" stroke-width="4" stroke-linecap="round"/>
                <circle cx="60" cy="60" r="5" fill="#0f172a"/>
            </svg>
            <div id="pitch_value" class="g-value">0.00 deg</div>
        </div>
        <div class="gauge">
            <div class="g-title">Roll</div>
            <svg viewBox="0 0 120 70" aria-label="Roll gauge">
                <path d="M10,60 A50,50 0 0 1 110,60" fill="none" stroke="#94a3b8" stroke-width="8"/>
                <line id="roll_needle" x1="60" y1="60" x2="60" y2="16" stroke="#0f766e" stroke-width="4" stroke-linecap="round"/>
                <circle cx="60" cy="60" r="5" fill="#0f172a"/>
            </svg>
            <div id="roll_value" class="g-value">0.00 deg</div>
        </div>
    </div>
</div>

<div class="card">
    <h3>Calibration niveau</h3>
    <p>Positionner le van sur une surface de reference puis faire la remise a zero.</p>
    <div class="actions">
        <button class="btn" onclick="captureZero()">Remise a zero</button>
        <button class="btn secondary" onclick="resetZero()">Annuler remise a zero</button>
        <button class="btn secondary" onclick="refreshLevel()">Rafraichir</button>
    </div>
    <div id="zero_status" class="status">Chargement...</div>
</div>

<script>
function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
}

function cm(mm) {
    return (Number(mm || 0) / 10.0).toFixed(1);
}

function gaugeNeedle(elementId, deg) {
    const clamped = clamp(Number(deg || 0), -20, 20);
    const angle = clamped * 4.5;
    const needle = document.getElementById(elementId);
    needle.setAttribute('transform', 'rotate(' + angle.toFixed(2) + ' 60 60)');
}

function renderLevel(data) {
    const pitch = Number(data.level_pitch_deg || 0);
    const roll = Number(data.level_roll_deg || 0);

    document.getElementById('lbl_fl').textContent = cm(data.level_wheel_fl_mm) + ' cm';
    document.getElementById('lbl_fr').textContent = cm(data.level_wheel_fr_mm) + ' cm';
    document.getElementById('lbl_rl').textContent = cm(data.level_wheel_rl_mm) + ' cm';
    document.getElementById('lbl_rr').textContent = cm(data.level_wheel_rr_mm) + ' cm';

    document.getElementById('pitch_value').textContent = pitch.toFixed(2) + ' deg';
    document.getElementById('roll_value').textContent = roll.toFixed(2) + ' deg';
    gaugeNeedle('pitch_needle', pitch);
    gaugeNeedle('roll_needle', roll);

    const rollVis = clamp(roll, -20, 20);
    const pitchVis = clamp(pitch, -20, 20);
    document.getElementById('van_wrap').style.transform =
        'translate(-50%, -50%) perspective(700px) rotateX(' + (-pitchVis).toFixed(2) + 'deg) rotateZ(' + rollVis.toFixed(2) + 'deg)';
}

async function refreshLevel() {
    try {
        const r = await fetch('/api/level_status');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('zero_status').textContent = 'Erreur lecture Level';
            return;
        }

        renderLevel(j);
        document.getElementById('zero_status').textContent =
            'Zero pitch=' + Number(j.level_zero_pitch_deg || 0).toFixed(2) +
            ' deg, zero roll=' + Number(j.level_zero_roll_deg || 0).toFixed(2) + ' deg';
    } catch (e) {
        document.getElementById('zero_status').textContent = 'Erreur reseau';
    }
}

async function captureZero() {
    document.getElementById('zero_status').textContent = 'Remise a zero en cours...';
    try {
        const r = await fetch('/api/level_zero_capture');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('zero_status').textContent = 'Echec remise a zero: ' + (j.error || 'unknown');
            return;
        }
        await refreshLevel();
        document.getElementById('zero_status').textContent = j.saved ? 'Remise a zero enregistree' : 'Remise a zero appliquee';
    } catch (e) {
        document.getElementById('zero_status').textContent = 'Echec remise a zero (reseau)';
    }
}

async function resetZero() {
    try {
        const r = await fetch('/api/level_zero_reset');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('zero_status').textContent = 'Echec annulation: ' + (j.error || 'unknown');
            return;
        }
        await refreshLevel();
        document.getElementById('zero_status').textContent = j.saved ? 'Zero reset enregistre' : 'Zero reset applique';
    } catch (e) {
        document.getElementById('zero_status').textContent = 'Echec annulation (reseau)';
    }
}

setInterval(refreshLevel, 1500);
refreshLevel();
</script>
</body>
</html>
)HTML";

const char DIAGNOSTICS_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Diagnostics</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
pre { white-space: pre-wrap; word-wrap: break-word; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
</style>
</head>
<body>
<h1>Diagnostics</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/level">Level</a>
    <a class="button" href="/config">Configuration</a>
    <a class="button" href="/gaz">Gaz 907</a>
    <a class="button" href="/update">Firmware Update</a>
</div>
<div class="card">
    <h3>Full Snapshot (Data + Config + HW)</h3>
    <pre id="diag">Loading...</pre>
</div>
<script>
async function refresh() {
    try {
        const r = await fetch('/api/full');
        const j = await r.json();
        document.getElementById('diag').textContent = JSON.stringify(j, null, 2);
    } catch (e) {
        document.getElementById('diag').textContent = 'Error loading diagnostics';
    }
}
setInterval(refresh, 1500);
refresh();
</script>
</body>
</html>
)HTML";

const char LOGS_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Logs</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
pre { white-space: pre-wrap; word-wrap: break-word; background: #101218; color: #d8dee9; padding: 12px; border-radius: 8px; min-height: 50vh; max-height: 70vh; overflow-y: auto; }
.controls { display: flex; gap: 8px; margin-top: 8px; }
.filters { display: flex; gap: 8px; margin-top: 8px; flex-wrap: wrap; }
select, input { border: 1px solid #bbb; border-radius: 6px; padding: 6px 8px; }
.status { font-size: 0.92em; color: #444; margin-top: 8px; }
button { border: 0; border-radius: 6px; padding: 8px 12px; cursor: pointer; }
</style>
</head>
<body>
<h1>Runtime Logs</h1>
<div class="card">
    <a class="button" href="/">Data</a>
</div>
<div class="card">
    <pre id="logs">Loading...</pre>
    <div class="filters">
        <label>Level
            <select id="level_filter" onchange="render()">
                <option value="ALL">ALL</option>
                <option value="D">D</option>
                <option value="I">I</option>
                <option value="W">W</option>
                <option value="E">E</option>
            </select>
        </label>
        <label>Search
            <input id="text_filter" type="text" placeholder="contains..." oninput="render()" />
        </label>
        <label>Max lines
            <input id="max_lines" type="number" min="100" max="5000" step="100" value="1200" onchange="applyMaxLines()" />
        </label>
    </div>
    <div class="controls">
        <button id="pause_btn" onclick="togglePause()">Pause</button>
        <button onclick="clearView()">Clear View</button>
        <button onclick="pullNow()">Refresh Now</button>
    </div>
    <div id="logs_status" class="status">--</div>
</div>
<script>
let lastSeq = 0;
let paused = false;
let maxStoredLines = 1200;
const lineStore = [];

function applyMaxLines() {
    const el = document.getElementById('max_lines');
    const parsed = Number(el.value || 1200);
    maxStoredLines = Math.max(100, Math.min(5000, Math.floor(parsed || 1200)));
    el.value = String(maxStoredLines);

    if (lineStore.length > maxStoredLines) {
        lineStore.splice(0, lineStore.length - maxStoredLines);
    }

    render();
}

function updateStatus(filteredCount) {
    const status = document.getElementById('logs_status');
    status.textContent =
        'stored=' + lineStore.length +
        ' shown=' + filteredCount +
        ' next_seq=' + lastSeq +
        (paused ? ' (paused)' : ' (live)');
}

function render() {
    const box = document.getElementById('logs');
    const levelFilter = document.getElementById('level_filter').value;
    const textFilter = document.getElementById('text_filter').value.trim().toLowerCase();

    const atBottom = (box.scrollTop + box.clientHeight + 8) >= box.scrollHeight;
    const filtered = lineStore.filter((e) => {
        if (levelFilter !== 'ALL' && e.level !== levelFilter) {
            return false;
        }
        if (textFilter && !e.line.toLowerCase().includes(textFilter)) {
            return false;
        }
        return true;
    });

    box.textContent = filtered.length ? filtered.map((e) => e.line).join('\n') + '\n' : '(no matching logs)\n';
    updateStatus(filtered.length);

    if (atBottom) {
        box.scrollTop = box.scrollHeight;
    }
}

function pushLine(level, line) {
    lineStore.push({ level, line });
    if (lineStore.length > maxStoredLines) {
        lineStore.splice(0, lineStore.length - maxStoredLines);
    }
}

function togglePause() {
    paused = !paused;
    document.getElementById('pause_btn').textContent = paused ? 'Resume' : 'Pause';
    updateStatus(lineStore.length);
}

function clearView() {
    lineStore.length = 0;
    render();
}

async function pullNow() {
    if (paused) {
        return;
    }

    try {
        const r = await fetch('/api/logs?after=' + encodeURIComponent(String(lastSeq)) + '&max=120');
        const payload = await r.json();
        const entries = payload.entries || [];

        if (lastSeq === 0 && entries.length === 0) {
            lineStore.length = 0;
            render();
            return;
        }

        for (const e of entries) {
            const ts = String(e.time || '00-00-00 00:00:00');
            const level = (e.level || '?');
            pushLine(level, '[' + ts + '][' + level + ']' + (e.msg || ''));
            lastSeq = Math.max(lastSeq, Number(e.seq || lastSeq));
        }

        if (payload.next_seq) {
            lastSeq = Math.max(lastSeq, Number(payload.next_seq) - 1);
        }

        render();
    } catch (err) {
        pushLine('E', '[error] failed to fetch logs');
        render();
    }
}

setInterval(pullNow, 700);
applyMaxLines();
pullNow();
</script>
</body>
</html>
)HTML";

