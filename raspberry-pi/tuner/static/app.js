const defaultGains = [
  [-2.0, -1.0, -2.8, -10.0, -1.2, -0.16],
  [-2.0,  1.0, -2.8, -10.0, -1.2,  0.16]
];

const metrics = [
  "valid", "balance", "fault", "fall",
  "fault_code", "fault_name",
  "cmd_age", "cmd_count",
  "imu", "imu_stale", "ori_age", "gyro_age",
  "ori_count", "gyro_count",
  "theta", "theta_dot", "theta_ddot", "v",
  "left_T", "right_T", "left_dT", "right_dT",
  "left_ref", "right_ref", "left_meas", "right_meas",
  "left_pwm", "right_pwm", "gain", "max_T"
];

const joystickDeadband = 0.05;
const joystickSendPeriodMs = 80;

let joystickCommand = {v: 0, yaw: 0};
let joystickLastSendMs = 0;
let joystickSendTimer = null;

function log(text) {
  const node = document.getElementById("log");
  const time = new Date().toLocaleTimeString();
  node.textContent = `[${time}] ${text}\n` + node.textContent;
}

function buildReadout() {
  const readout = document.getElementById("readout");
  readout.innerHTML = "";
  for (const key of metrics) {
    const box = document.createElement("div");
    box.className = "metric";
    box.innerHTML = `<span>${key}</span><strong id="m_${key}">-</strong>`;
    readout.appendChild(box);
  }
}

function buildGainMatrix() {
  const matrix = document.getElementById("gain_matrix");
  matrix.textContent = "";
  matrix.appendChild(document.createElement("div"));

  for (let col = 0; col < 6; col++) {
    const heading = document.createElement("div");
    heading.className = "head";
    heading.textContent = `Z${col}`;
    matrix.appendChild(heading);
  }

  for (let row = 0; row < 2; row++) {
    const label = document.createElement("div");
    label.className = "head";
    label.textContent = row === 0 ? "left" : "right";
    matrix.appendChild(label);

    for (let col = 0; col < 6; col++) {
      const input = document.createElement("input");
      input.id = `gain_${row}_${col}`;
      input.type = "number";
      input.step = "0.001";
      input.value = defaultGains[row][col];
      matrix.appendChild(input);
    }
  }
}

async function api(path, options) {
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok || data.ok === false) {
    throw new Error(data.error || "request failed");
  }
  return data;
}

async function sendCommand(command) {
  const data = await api("/api/command", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({command})
  });
  log(`${command} -> ${data.response || "OK"}`);
  return data;
}

function sendValue(prefix, inputId) {
  const value = document.getElementById(inputId).value;
  return sendCommand(`${prefix} ${value}`);
}

function applyMotionCommand() {
  const v = document.getElementById("v_cmd").value;
  const yaw = document.getElementById("yaw_cmd").value;
  return sendCommand(`balance command ${v} ${yaw}`);
}

function formatCommandValue(value) {
  if (Math.abs(value) < 0.0005) {
    return "0";
  }

  return value.toFixed(3);
}

function applyJoystickVector(x, y) {
  const driveSpeed =
    Number(document.getElementById("drive_speed").value) || 0;
  const turnSpeed =
    Number(document.getElementById("turn_speed").value) || 0;
  const v =
    formatCommandValue(-y * driveSpeed);
  const yaw =
    formatCommandValue(-x * turnSpeed);

  joystickCommand = {v, yaw};

  document.getElementById("v_cmd").value = v;
  document.getElementById("yaw_cmd").value = yaw;

  scheduleJoystickSend(false);
}

function sendJoystickCommand(force) {
  const now = Date.now();
  const elapsedMs =
    now - joystickLastSendMs;

  if (!force && elapsedMs < joystickSendPeriodMs) {
    scheduleJoystickSend(true);
    return;
  }

  joystickLastSendMs = now;
  sendCommand(`balance command ${joystickCommand.v} ${joystickCommand.yaw}`)
    .catch((error) => log(`joystick command failed: ${error.message}`));
}

function scheduleJoystickSend(deferOnly) {
  if (!deferOnly) {
    sendJoystickCommand(false);
    return;
  }

  if (joystickSendTimer !== null) {
    return;
  }

  const waitMs =
    Math.max(0, joystickSendPeriodMs - (Date.now() - joystickLastSendMs));

  joystickSendTimer = window.setTimeout(() => {
    joystickSendTimer = null;
    sendJoystickCommand(true);
  }, waitMs);
}

function centerJoystick(sendStop) {
  const knob =
    document.getElementById("joystick_knob");

  knob.style.transform =
    "translate(-50%, -50%)";

  joystickCommand = {v: "0", yaw: "0"};
  document.getElementById("v_cmd").value = "0";
  document.getElementById("yaw_cmd").value = "0";

  if (sendStop) {
    sendJoystickCommand(true);
  }
}

function updateJoystick(pointerEvent) {
  const base =
    document.getElementById("joystick");
  const knob =
    document.getElementById("joystick_knob");
  const rect =
    base.getBoundingClientRect();
  const radius =
    rect.width / 2;
  let x =
    (pointerEvent.clientX - rect.left - radius) / radius;
  let y =
    (pointerEvent.clientY - rect.top - radius) / radius;
  const magnitude =
    Math.hypot(x, y);

  if (magnitude > 1) {
    x /= magnitude;
    y /= magnitude;
  }

  if (Math.hypot(x, y) < joystickDeadband) {
    x = 0;
    y = 0;
  }

  knob.style.transform =
    `translate(calc(-50% + ${x * radius}px), calc(-50% + ${y * radius}px))`;

  applyJoystickVector(x, y);
}

function bindJoystick() {
  const base =
    document.getElementById("joystick");

  base.addEventListener("pointerdown", (event) => {
    base.setPointerCapture(event.pointerId);
    base.classList.add("active");
    updateJoystick(event);
  });

  base.addEventListener("pointermove", (event) => {
    if (!base.hasPointerCapture(event.pointerId)) {
      return;
    }

    updateJoystick(event);
  });

  for (const eventName of ["pointerup", "pointercancel"]) {
    base.addEventListener(eventName, (event) => {
      if (base.hasPointerCapture(event.pointerId)) {
        base.releasePointerCapture(event.pointerId);
      }

      base.classList.remove("active");
      centerJoystick(true);
    });
  }
}

function resetStm32() {
  return sendCommand("system reset");
}

function bindControls() {
  document.getElementById("balance_start")
    .addEventListener("click", () => sendCommand("balance start"));

  document.getElementById("balance_stop")
    .addEventListener("click", () => sendCommand("balance stop"));

  document.getElementById("actuator_stop")
    .addEventListener("click", () => sendCommand("actuator stop"));

  document.getElementById("system_reset")
    .addEventListener("click", resetStm32);

  document.getElementById("motion_stop")
    .addEventListener("click", () => centerJoystick(true));

  document.getElementById("apply_motion")
    .addEventListener("click", applyMotionCommand);

  document.getElementById("apply_gains")
    .addEventListener("click", applyAllGains);

  for (const button of document.querySelectorAll("[data-command-prefix]")) {
    button.addEventListener("click", () => {
      sendValue(button.dataset.commandPrefix, button.dataset.inputId);
    });
  }
}

async function applyAllGains() {
  for (let row = 0; row < 2; row++) {
    const side = row === 0 ? "left" : "right";
    for (let col = 0; col < 6; col++) {
      const value = document.getElementById(`gain_${row}_${col}`).value;
      await sendCommand(`balance gain ${side} ${col} ${value}`);
    }
  }
}

let telemetryPollActive = false;
let lastAck = "";

async function pollTelemetry() {
  if (telemetryPollActive) {
    return;
  }

  telemetryPollActive = true;
  const status = document.getElementById("connection");

  try {
    const data = await api("/api/telemetry");
    status.textContent = "connected";
    status.className = "status ok";
    if (
      data.command &&
      data.command.ack &&
      data.command.ack !== lastAck
    ) {
      lastAck = data.command.ack;
      log(`STM ${data.command.ack}`);
    }

    for (const key of metrics) {
      const node = document.getElementById(`m_${key}`);
      if (
        node &&
        Object.prototype.hasOwnProperty.call(data.telemetry, key) &&
        data.telemetry[key] !== null
      ) {
        node.textContent = data.telemetry[key];
      }
    }
  } catch (error) {
    status.textContent = error.message;
    status.className = "status bad";
  } finally {
    telemetryPollActive = false;
  }
}

buildReadout();
buildGainMatrix();
bindJoystick();
bindControls();
centerJoystick(false);
pollTelemetry();
setInterval(pollTelemetry, 300);
