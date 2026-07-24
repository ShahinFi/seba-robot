const stateMetrics = [
  ["Mode", "balance"],
  ["Fault", "fault_name"],
  ["Pitch [rad]", "theta"],
  ["Pitch rate [rad/s]", "theta_dot"],
  ["Drive command [m/s]", "v_cmd"],
  ["Turn command [rad/s]", "yaw_cmd"],
  ["Left current [mA]", "left_meas"],
  ["Right current [mA]", "right_meas"],
  ["Left PWM", "left_pwm"],
  ["Right PWM", "right_pwm"]
];

const joystickDeadband = 0.05;
const joystickSendPeriodMs = 80;
const resetHoldMs = 1500;

let joystickCommand = {v: "0", yaw: "0"};
let joystickHoldTimer = null;
let lastAck = "";
let telemetryPollActive = false;
let resetTimer = null;

async function api(path, options) {
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok || data.ok === false) {
    throw new Error(data.error || "request failed");
  }
  return data;
}

async function sendCommand(command, options = {}) {
  const shouldLog =
    options.log !== false;

  return api("/api/command", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({
      command,
      log: shouldLog
    })
  });
}

function buildStateGrid() {
  const grid = document.getElementById("state_grid");
  grid.textContent = "";

  for (const [label, key] of stateMetrics) {
    const item = document.createElement("div");
    item.className = "metric";
    item.innerHTML = `<span>${label}</span><strong id="m_${key}">-</strong>`;
    grid.appendChild(item);
  }
}

function appendEvent(text) {
  const node = document.getElementById("events");
  const time = new Date().toLocaleTimeString();
  const wasAtBottom =
    node.scrollTop + node.clientHeight >=
    node.scrollHeight - 4;

  node.textContent += `[${time}] ${text}\n`;

  if (wasAtBottom) {
    node.scrollTop = node.scrollHeight;
  }
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

  joystickCommand = {
    v: formatCommandValue(-y * driveSpeed),
    yaw: formatCommandValue(-x * turnSpeed)
  };
}

function sendJoystickCommand() {
  sendCommand(
    `balance command ${joystickCommand.v} ${joystickCommand.yaw}`,
    {log: false}
  ).catch(() => {});
}

function startJoystickHold() {
  if (joystickHoldTimer !== null) {
    return;
  }

  sendJoystickCommand();
  joystickHoldTimer = window.setInterval(
    sendJoystickCommand,
    joystickSendPeriodMs
  );
}

function stopJoystickHold() {
  if (joystickHoldTimer === null) {
    return;
  }

  window.clearInterval(joystickHoldTimer);
  joystickHoldTimer = null;
}

function centerJoystick(sendStop) {
  const knob = document.getElementById("joystick_knob");

  knob.style.transform = "translate(-50%, -50%)";
  joystickCommand = {v: "0", yaw: "0"};

  if (sendStop) {
    sendJoystickCommand();
  }
}

function updateJoystick(pointerEvent) {
  const base = document.getElementById("joystick");
  const knob = document.getElementById("joystick_knob");
  const rect = base.getBoundingClientRect();
  const radius = rect.width / 2;
  let x = (pointerEvent.clientX - rect.left - radius) / radius;
  let y = (pointerEvent.clientY - rect.top - radius) / radius;
  const magnitude = Math.hypot(x, y);

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
  const base = document.getElementById("joystick");

  base.addEventListener("pointerdown", (event) => {
    base.setPointerCapture(event.pointerId);
    base.classList.add("active");
    updateJoystick(event);
    startJoystickHold();
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
      stopJoystickHold();
      centerJoystick(true);
    });
  }
}

function emergencyStop() {
  centerJoystick(false);
  stopJoystickHold();
  sendCommand("actuator stop").catch((error) => appendEvent(error.message));
}

function bindResetHold() {
  const button = document.getElementById("reset_stm32");

  function cancelReset() {
    if (resetTimer !== null) {
      window.clearTimeout(resetTimer);
      resetTimer = null;
    }
    button.classList.remove("holding");
  }

  button.addEventListener("pointerdown", () => {
    if (resetTimer !== null) {
      return;
    }

    button.classList.add("holding");
    resetTimer = window.setTimeout(() => {
      resetTimer = null;
      button.classList.remove("holding");
      sendCommand("system reset").catch((error) => appendEvent(error.message));
    }, resetHoldMs);
  });

  for (const eventName of ["pointerup", "pointerleave", "pointercancel"]) {
    button.addEventListener(eventName, cancelReset);
  }
}

function bindControls() {
  document.getElementById("balance_start")
    .addEventListener("click", () => sendCommand("balance start"));

  document.getElementById("balance_stop")
    .addEventListener("click", () => sendCommand("balance stop"));

  document.getElementById("emergency_stop")
    .addEventListener("click", emergencyStop);

  document.getElementById("motion_stop")
    .addEventListener("click", () => {
      centerJoystick(false);
      sendCommand("balance command 0 0").catch(
        (error) => appendEvent(error.message)
      );
    });

  bindResetHold();
}

function setChip(id, text, className) {
  const node = document.getElementById(id);
  node.textContent = text;
  node.className = `chip ${className}`;
}

function updateFaultBanner(telemetry) {
  const banner = document.getElementById("fault_banner");
  const text = document.getElementById("fault_text");
  const active =
    Number(telemetry.fault) !== 0;

  banner.classList.toggle("hidden", !active);
  text.textContent =
    telemetry.fault_name || "unknown";
}

function updateState(telemetry) {
  const balanceActive =
    Number(telemetry.balance) !== 0;
  const faultActive =
    Number(telemetry.fault) !== 0;
  const theta =
    telemetry.theta ?? "-";

  setChip(
    "mode",
    faultActive ? "faulted" : (balanceActive ? "balancing" : "stopped"),
    faultActive ? "bad" : (balanceActive ? "ok" : "muted")
  );
  setChip("pitch_chip", `theta ${theta}`, faultActive ? "bad" : "muted");
  updateFaultBanner(telemetry);

  for (const [, key] of stateMetrics) {
    const node = document.getElementById(`m_${key}`);
    if (!node) {
      continue;
    }

    if (key === "balance") {
      node.textContent =
        faultActive ? "faulted" : (balanceActive ? "balancing" : "stopped");
      continue;
    }

    if (
      Object.prototype.hasOwnProperty.call(telemetry, key) &&
      telemetry[key] !== null
    ) {
      node.textContent = telemetry[key];
    }
  }
}

async function pollTelemetry() {
  if (telemetryPollActive) {
    return;
  }

  telemetryPollActive = true;

  try {
    const data = await api("/api/telemetry");
    setChip(
      "connection",
      data.stale ? "stale" : "connected",
      data.stale ? "warn" : "ok"
    );

    if (
      data.command &&
      data.command.ack &&
      data.command.ack !== lastAck
    ) {
      lastAck = data.command.ack;
      appendEvent(`STM ${data.command.ack}`);
    }

    if (Array.isArray(data.logs)) {
      for (const line of data.logs) {
        appendEvent(line);
      }
    }

    updateState(data.telemetry);
  } catch (error) {
    setChip("connection", error.message, "bad");
  } finally {
    telemetryPollActive = false;
  }
}

buildStateGrid();
bindJoystick();
bindControls();
centerJoystick(false);
pollTelemetry();
setInterval(pollTelemetry, 300);
