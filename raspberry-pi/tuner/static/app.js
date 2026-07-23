const defaultGains = [
  [-2.0, -1.0, -2.8, -10.0, -1.2, -0.16],
  [-2.0,  1.0, -2.8, -10.0, -1.2,  0.16]
];

const metrics = [
  "valid", "balance", "fault", "fall",
  "theta", "theta_dot", "theta_ddot", "v",
  "left_T", "right_T", "left_dT", "right_dT",
  "left_ref", "right_ref", "left_meas", "right_meas",
  "left_pwm", "right_pwm", "gain", "max_T"
];

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
bindControls();
pollTelemetry();
setInterval(pollTelemetry, 300);
