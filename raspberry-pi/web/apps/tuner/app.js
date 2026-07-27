const defaultGains = [
  [-2.0, -1.0, -2.3, -8.0, -0.7, -0.16],
  [-2.0,  1.0, -2.3, -8.0, -0.7,  0.16]
];

const primaryMetrics = [
  "valid", "balance", "fault_name", "fall"
];

const telemetryLabels = {
  valid: "valid",
  balance: "balance",
  fault_name: "fault name",
  fall: "fall",
  theta: "theta [rad]",
  theta_dot: "theta_dot [rad/s]",
  theta_ddot: "theta_ddot [rad/s^2]",
  v: "v [m/s]",
  v_dot: "v_dot [m/s^2]",
  psi_dot: "psi_dot [rad/s]",
  psi_ddot: "psi_ddot [rad/s^2]",
  v_cmd: "v_cmd [m/s]",
  yaw_cmd: "yaw_cmd [rad/s]",
  cmd_age: "cmd_age [ms]",
  cmd_count: "cmd_count",
  left_T: "left_T [mNm]",
  right_T: "right_T [mNm]",
  left_dT: "left_dT [mNm/s]",
  right_dT: "right_dT [mNm/s]",
  gain: "gain [x]",
  max_T: "max_T [mNm]",
  act: "act",
  act_fault: "act_fault",
  act_read_failed: "act_read_failed",
  left_ref: "left_ref [mA]",
  right_ref: "right_ref [mA]",
  left_meas: "left_meas [mA]",
  right_meas: "right_meas [mA]",
  left_err: "left_err [mA]",
  right_err: "right_err [mA]",
  left_int: "left_int [mV]",
  right_int: "right_int [mV]",
  left_pwm: "left_pwm [permille]",
  right_pwm: "right_pwm [permille]",
  imu: "imu",
  imu_stale: "imu_stale",
  enc: "enc",
  updates: "updates",
  ori_age: "ori_age [ms]",
  gyro_age: "gyro_age [ms]",
  ori_count: "ori_count",
  gyro_count: "gyro_count",
  fault: "fault",
  fault_code: "fault_code"
};

const telemetryGroups = [
  {
    title: "State",
    keys: ["theta", "theta_dot", "theta_ddot", "v", "v_dot", "psi_dot", "psi_ddot"]
  },
  {
    title: "Command",
    keys: ["v_cmd", "yaw_cmd", "cmd_age", "cmd_count"]
  },
  {
    title: "Control",
    keys: ["left_T", "right_T", "left_dT", "right_dT", "gain", "max_T"]
  },
  {
    title: "Actuator",
    keys: [
      "act", "act_fault", "act_read_failed",
      "left_ref", "right_ref",
      "left_meas", "right_meas",
      "left_err", "right_err",
      "left_int", "right_int",
      "left_pwm", "right_pwm"
    ]
  },
  {
    title: "Estimator",
    keys: ["imu", "imu_stale", "enc", "updates"]
  },
  {
    title: "IMU Reports",
    keys: ["ori_age", "gyro_age", "ori_count", "gyro_count"]
  },
  {
    title: "Fault",
    keys: ["fault", "fault_code"]
  }
];

const joystickDeadband = 0.05;
const joystickSendPeriodMs = 80;
const resetHoldMs = 1500;

let joystickCommand = {v: 0, yaw: 0};
let joystickHoldTimer = null;
let resetTimer = null;
let csrfToken = "";
let authenticated = false;

function writeLog(nodeId, text) {
  const node = document.getElementById(nodeId);
  const time = new Date().toLocaleTimeString();
  const wasAtBottom =
    node.scrollTop + node.clientHeight >=
    node.scrollHeight - 4;

  node.textContent += `[${time}] ${text}\n`;

  if (wasAtBottom) {
    node.scrollTop = node.scrollHeight;
  }
}

function logCommand(text) {
  writeLog("command_log", text);
}

function logStm(text) {
  writeLog("stm_log", text);
}

function setChip(id, text, className) {
  const node = document.getElementById(id);

  node.textContent = text;
  node.className = `chip ${className}`;
}

function buildReadout() {
  const readout = document.getElementById("readout");
  readout.innerHTML = "";

  const statusGrid = document.createElement("div");
  statusGrid.className = "telemetry-status";

  for (const key of primaryMetrics) {
    const box = document.createElement("div");
    box.className = "metric";
    box.innerHTML = `<span>${telemetryLabels[key]}</span><strong id="m_${key}">-</strong>`;
    statusGrid.appendChild(box);
  }

  const groupColumns = document.createElement("div");
  groupColumns.className = "telemetry-groups";

  for (const group of telemetryGroups) {
    const section = document.createElement("div");
    section.className = "telemetry-group";
    section.innerHTML = `<h3>${group.title}</h3>`;

    for (const key of group.keys) {
      const row = document.createElement("div");
      row.className = "telemetry-row";
      row.innerHTML = `<span>${telemetryLabels[key]}</span><strong id="m_${key}">-</strong>`;
      section.appendChild(row);
    }

    groupColumns.appendChild(section);
  }

  readout.appendChild(statusGrid);
  readout.appendChild(groupColumns);
}

function buildGainMatrix() {
  const matrix = document.getElementById("gain_matrix");
  matrix.textContent = "";
  matrix.appendChild(document.createElement("div"));

  /*
   * Matrix order matches the STM32 balance config command:
   * left Z0...Z5, then right Z0...Z5.
   */
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
  const requestOptions = options || {};
  requestOptions.headers = requestOptions.headers || {};
  if (csrfToken && requestOptions.method === "POST") {
    requestOptions.headers["X-SEBA-CSRF"] = csrfToken;
  }

  const response = await fetch(path, requestOptions);
  const data = await response.json();
  if (!response.ok || data.ok === false) {
    throw new Error(data.error || "request failed");
  }
  return data;
}

function showAuth(message) {
  const overlay = document.getElementById("auth_overlay");
  const messageNode = document.getElementById("auth_message");

  authenticated = false;
  csrfToken = "";
  messageNode.textContent = message;
  overlay.classList.remove("hidden");
  document.getElementById("logout").classList.add("hidden");
  stopJoystickHold();
  centerJoystick(false);
}

function hideAuth(role) {
  authenticated = true;
  document.getElementById("auth_overlay").classList.add("hidden");
  document.getElementById("logout").classList.remove("hidden");
  logStm(`web session unlocked as ${role}`);
}

async function refreshAuth() {
  const data = await api("/api/auth");
  const auth = data.auth;

  if (!auth.configured) {
    showAuth("Web authentication is not configured. Run: bash raspberry-pi/install.sh auth");
    return false;
  }

  if (!auth.authenticated) {
    showAuth("Engineer access is required.");
    return false;
  }

  if (auth.role !== "engineer") {
    showAuth("Engineer access is required.");
    return false;
  }

  csrfToken = auth.csrf;
  hideAuth(auth.role);
  return true;
}

async function login(event) {
  event.preventDefault();

  try {
    const data = await api("/api/login", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({
        role: document.getElementById("auth_role").value,
        password: document.getElementById("auth_password").value
      })
    });
    csrfToken = data.csrf;
    document.getElementById("auth_password").value = "";
    hideAuth(data.role);
    pollTelemetry();
  } catch (error) {
    showAuth(error.message);
  }
}

async function logout() {
  stopJoystickHold();
  centerJoystick(false);
  await sendCommand("balance command 0 0", {log: false}).catch(() => {});

  try {
    await api("/api/logout", {method: "POST"});
  } catch (_) {
  }

  showAuth("Session closed.");
}

async function sendCommand(command, options = {}) {
  const shouldLog =
    options.log !== false;

  try {
    const data = await api("/api/command", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({
        command,
        log: shouldLog
      })
    });
    if (shouldLog) {
      logCommand(`${command} -> ${data.response || "OK"}`);
    }
    return data;
  } catch (error) {
    if (shouldLog) {
      logCommand(`${command} -> ERROR: ${error.message}`);
    }
    throw error;
  }
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

  /*
   * Screen coordinates have positive Y downward. The STM32 command convention
   * uses positive drive velocity for forward motion and positive yaw to turn left.
   */
  const v =
    formatCommandValue(-y * driveSpeed);
  const yaw =
    formatCommandValue(-x * turnSpeed);

  joystickCommand = {v, yaw};

  document.getElementById("v_cmd").value = v;
  document.getElementById("yaw_cmd").value = yaw;
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

  /*
   * Motion commands are a heartbeat. The STM32 clears stale motion commands,
   * so the page repeats the active joystick command while the pointer is held.
   */
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
  const knob =
    document.getElementById("joystick_knob");

  knob.style.transform =
    "translate(-50%, -50%)";

  joystickCommand = {v: "0", yaw: "0"};
  document.getElementById("v_cmd").value = "0";
  document.getElementById("yaw_cmd").value = "0";

  /*
   * Releasing the joystick sends one explicit zero command before the browser
   * stops the heartbeat.
   */
  if (sendStop) {
    sendJoystickCommand();
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

function stopMotion() {
  centerJoystick(false);
  return sendCommand("balance command 0 0");
}

function applyActuatorSettings() {
  /*
   * The tuner applies actuator settings as one STM32 command so one button
   * press maps to one acknowledged protocol transaction.
   */
  const values = [
    "act_kp",
    "act_ki",
    "act_max_current",
    "act_max_pwm",
    "act_integral",
    "act_ktw"
  ].map((inputId) => document.getElementById(inputId).value);

  return sendCommand(`actuator config ${values.join(" ")}`);
}

function applyBalanceLoop() {
  /*
   * The balance apply button sends the gain scale, torque limit, and complete
   * RSLQR matrix together so the controller is not tuned one field at a time.
   */
  const values = [
    document.getElementById("gain_scale").value,
    document.getElementById("max_torque").value
  ];

  for (let row = 0; row < 2; row++) {
    for (let col = 0; col < 6; col++) {
      values.push(
        document.getElementById(`gain_${row}_${col}`).value
      );
    }
  }

  return sendCommand(`balance config ${values.join(" ")}`);
}

function bindResetHold() {
  const button = document.getElementById("system_reset");

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
      sendCommand("system reset");
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

  document.getElementById("motion_stop")
    .addEventListener("click", stopMotion);

  document.getElementById("apply_motion")
    .addEventListener("click", applyMotionCommand);

  document.getElementById("apply_actuator_settings")
    .addEventListener("click", applyActuatorSettings);

  document.getElementById("apply_balance_loop")
    .addEventListener("click", applyBalanceLoop);

  bindResetHold();
}

let telemetryPollActive = false;
let lastAck = "";

async function pollTelemetry() {
  if (!authenticated) {
    return;
  }

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

    const faultActive =
      Number(data.telemetry.fault) !== 0;
    const balanceActive =
      Number(data.telemetry.balance) !== 0;

    setChip(
      "mode",
      faultActive ? "faulted" : (balanceActive ? "balancing" : "stopped"),
      faultActive ? "bad" : (balanceActive ? "ok" : "muted")
    );

    if (
      data.command &&
      data.command.ack &&
      data.command.ack !== lastAck
    ) {
      lastAck = data.command.ack;
      logCommand(`STM ${data.command.ack}`);
    }

    if (Array.isArray(data.logs)) {
      for (const line of data.logs) {
        logStm(line);
      }
    }

    for (const key of [...primaryMetrics, ...telemetryGroups.flatMap((group) => group.keys)]) {
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
    setChip("connection", error.message, "bad");
    if (
      error.message === "login required" ||
      error.message === "engineer access required" ||
      error.message === "invalid csrf token"
    ) {
      showAuth(error.message);
    }
  } finally {
    telemetryPollActive = false;
  }
}

async function initialize() {
  buildReadout();
  buildGainMatrix();
  bindJoystick();
  bindControls();
  centerJoystick(false);
  document.getElementById("auth_form").addEventListener("submit", login);
  document.getElementById("logout").addEventListener("click", logout);

  try {
    if (await refreshAuth()) {
      pollTelemetry();
    }
  } catch (error) {
    showAuth(error.message);
  }

  setInterval(pollTelemetry, 300);
}

initialize();
