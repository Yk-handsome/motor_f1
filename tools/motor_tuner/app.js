"use strict";

const MAX_CURRENT_A = 2.0;
const TELEMETRY_ABS_LIMIT_A = 20.0;
const MAX_SAMPLES = 600;
const chartColors = {
  iqRef: "#d53f8c",
  iq: "#137cbd",
  idRef: "#d98700",
  id: "#0b886b",
  grid: "#dce4e8",
  axis: "#8a9aa6",
  label: "#5d6c76"
};

const state = {
  port: null,
  reader: null,
  writer: null,
  reading: false,
  lineBuffer: "",
  samples: [],
  rxLines: 0,
  telemetryLines: 0,
  invalidLines: 0,
  telemetryTimes: [],
  resizeObserver: null,
  txQueue: [],
  txBusy: false,
  operationBusy: false,
  eventCount: 0
};

// The current text-command firmware owns only one command buffer. This gap lets
// the main loop consume a CR/LF-terminated command before the next one arrives.
const TX_COMMAND_GAP_MS = 35;

const elements = {
  connectButton: document.querySelector("#connectButton"),
  baudRate: document.querySelector("#baudRate"),
  connectionStatus: document.querySelector("#connectionStatus"),
  connectionText: document.querySelector("#connectionText"),
  emergencyStop: document.querySelector("#emergencyStop"),
  mode: document.querySelector("#mode"),
  applyMode: document.querySelector("#applyMode"),
  targetIq: document.querySelector("#targetIq"),
  targetId: document.querySelector("#targetId"),
  targetSpeed: document.querySelector("#targetSpeed"),
  targetPosition: document.querySelector("#targetPosition"),
  applyTargets: document.querySelector("#applyTargets"),
  applyLimits: document.querySelector("#applyLimits"),
  limitTorque: document.querySelector("#limitTorque"),
  limitSpeed: document.querySelector("#limitSpeed"),
  readConfig: document.querySelector("#readConfig"),
  sendManual: document.querySelector("#sendManual"),
  manualCommand: document.querySelector("#manualCommand"),
  clearLog: document.querySelector("#clearLog"),
  deviceLog: document.querySelector("#deviceLog"),
  eventLog: document.querySelector("#eventLog"),
  eventCount: document.querySelector("#eventCount"),
  eventRows: document.querySelector("#eventRows"),
  eventState: document.querySelector("#eventState"),
  eventMode: document.querySelector("#eventMode"),
  eventHeartbeat: document.querySelector("#eventHeartbeat"),
  clearChart: document.querySelector("#clearChart"),
  sampleWindow: document.querySelector("#sampleWindow"),
  telemetryChart: document.querySelector("#telemetryChart"),
  rxLines: document.querySelector("#rxLines"),
  telemetryLines: document.querySelector("#telemetryLines"),
  invalidLines: document.querySelector("#invalidLines"),
  telemetryRate: document.querySelector("#telemetryRate"),
  iqRef: document.querySelector("#iqRef"),
  iqActual: document.querySelector("#iqActual"),
  idRef: document.querySelector("#idRef"),
  idActual: document.querySelector("#idActual"),
  currentWarning: document.querySelector("#currentWarning"),
  pidApplyButtons: [...document.querySelectorAll(".pid-apply")]
};

function setConnected(connected, message = "") {
  elements.connectionStatus.dataset.state = connected ? "connected" : (message ? "error" : "offline");
  elements.connectionText.textContent = connected ? "串口已连接" : (message || "未连接");
  elements.connectButton.textContent = connected ? "断开连接" : "选择串口并连接";
  const controlled = [elements.emergencyStop, elements.applyMode, elements.applyTargets, elements.applyLimits, elements.readConfig, elements.sendManual, ...elements.pidApplyButtons];
  controlled.forEach((item) => { item.disabled = !connected; });
}

function addLog(line) {
  const existing = elements.deviceLog.textContent.split("\n").filter(Boolean);
  existing.push(line);
  elements.deviceLog.textContent = existing.slice(-180).join("\n");
  elements.deviceLog.scrollTop = elements.deviceLog.scrollHeight;
}

const EVENT_LABELS = {
  uart_dma_ready: "串口 DMA 就绪",
  uart_dma_error: "串口 DMA 错误",
  heartbeat: "运行心跳",
  motor_stopped: "电机已停机",
  mode_changed: "控制模式已切换",
  pid_applied: "PID 参数已应用",
  pid_rejected_busy: "PID 被拒绝：电机仍在运行",
  target_applied: "目标值已应用",
  limit_applied: "安全限制已应用",
  command_error: "命令错误"
};

const MODE_LABELS = {
  null: "停机",
  torque: "电流 / 力矩",
  speed: "速度",
  speed_torque: "速度 + 力矩限制",
  position: "位置",
  position_speed_torque: "位置 + 速度 + 力矩限制"
};

function eventNumber(value) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(3) : "--";
}

function eventValue(event, name, unit) {
  const explicit = event[`${name}_${unit}`];
  if (explicit !== undefined) return explicit;
  return event[name];
}

function addEvent(event) {
  state.eventCount += 1;
  elements.eventCount.textContent = `${state.eventCount} 条`;
  const existing = elements.eventLog.textContent.split("\n").filter(Boolean);
  existing.push(JSON.stringify(event));
  elements.eventLog.textContent = existing.slice(-100).join("\n");
  elements.eventLog.scrollTop = elements.eventLog.scrollHeight;

  const label = EVENT_LABELS[event.event] || event.event || "未知事件";
  elements.eventState.textContent = label;
  elements.eventMode.textContent = MODE_LABELS[event.mode] || event.mode || "--";
  if (event.event === "heartbeat") {
    elements.eventHeartbeat.textContent = `tick ${event.tick ?? "--"} · Iq ${eventNumber(eventValue(event, "iq", "a"))} A`;
  }

  const row = document.createElement("tr");
  const values = [
    event.tick === undefined ? "--" : `${event.tick} ms`,
    label,
    MODE_LABELS[event.mode] || event.mode || "--",
    `${eventNumber(eventValue(event, "iq_ref", "a"))} / ${eventNumber(eventValue(event, "iq", "a"))} A`,
    `${eventNumber(eventValue(event, "id_ref", "a"))} / ${eventNumber(eventValue(event, "id", "a"))} A`
  ];
  values.forEach((value, index) => {
    const cell = document.createElement("td");
    cell.textContent = value;
    if (index === 1) cell.title = event.event || "";
    row.appendChild(cell);
  });
  elements.eventRows.prepend(row);
  while (elements.eventRows.children.length > 30) elements.eventRows.lastElementChild.remove();
}

function setOperationBusy(busy) {
  state.operationBusy = busy;
  elements.pidApplyButtons.forEach((button) => { button.disabled = !state.port || busy; });
  elements.applyMode.disabled = !state.port || busy;
  elements.applyTargets.disabled = !state.port || busy;
  elements.applyLimits.disabled = !state.port || busy;
  elements.readConfig.disabled = !state.port || busy;
  elements.sendManual.disabled = !state.port || busy;
  elements.emergencyStop.disabled = !state.port;
}

function updateMetrics() {
  elements.rxLines.textContent = state.rxLines;
  elements.telemetryLines.textContent = state.telemetryLines;
  elements.invalidLines.textContent = state.invalidLines;
  const now = performance.now();
  state.telemetryTimes = state.telemetryTimes.filter((value) => now - value < 1000);
  elements.telemetryRate.textContent = `${state.telemetryTimes.length} Hz`;
}

function finiteInput(element) {
  const value = Number.parseFloat(element.value);
  if (!Number.isFinite(value)) {
    throw new Error(`“${element.closest("label")?.textContent?.trim() || "输入值"}”必须是数字。`);
  }
  return value;
}

function wait(milliseconds) {
  return new Promise((resolve) => window.setTimeout(resolve, milliseconds));
}

function send(command, priority = false) {
  if (!state.writer) {
    addLog("[UI] 串口未连接，命令未发送。");
    return Promise.resolve(false);
  }
  return new Promise((resolve) => {
    const item = { command: command.trim(), resolve };
    if (priority) state.txQueue.unshift(item); else state.txQueue.push(item);
    drainTransmitQueue();
  });
}

async function drainTransmitQueue() {
  if (state.txBusy || !state.writer) return;
  state.txBusy = true;
  while (state.txQueue.length > 0 && state.writer) {
    const item = state.txQueue.shift();
    try {
      addLog(`> ${item.command}`);
      await state.writer.write(new TextEncoder().encode(`${item.command}\r\n`));
      item.resolve(true);
      await wait(TX_COMMAND_GAP_MS);
    } catch (error) {
      addLog(`[TX ERROR] ${error.message}`);
      item.resolve(false);
    }
  }
  state.txBusy = false;
}

async function connect() {
  if (!("serial" in navigator)) {
    addLog("[UI] 浏览器不支持 Web Serial，请用 Edge 或 Chrome。\n");
    setConnected(false, "浏览器不支持 Web Serial");
    return;
  }
  try {
    state.port = await navigator.serial.requestPort();
    await state.port.open({ baudRate: Number(elements.baudRate.value), dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
    state.writer = state.port.writable.getWriter();
    state.reading = true;
    setConnected(true);
    addLog(`[UI] 已连接，${elements.baudRate.value} bps`);
    readLoop();
    await send("show");
  } catch (error) {
    cleanupPort();
    setConnected(false, `连接失败：${error.message}`);
    addLog(`[CONNECT ERROR] ${error.message}`);
  }
}

async function disconnect(sendStop = true) {
  if (!state.port) return;
  try {
    if (sendStop && state.writer) await send("mode null", true);
    state.reading = false;
    if (state.reader) {
      await state.reader.cancel();
    }
    if (state.writer) {
      state.writer.releaseLock();
      state.writer = null;
    }
    await state.port.close();
    addLog("[UI] 串口已断开。" );
  } catch (error) {
    addLog(`[DISCONNECT ERROR] ${error.message}`);
  } finally {
    cleanupPort();
    setConnected(false);
  }
}

function cleanupPort() {
  if (state.reader) {
    try { state.reader.releaseLock(); } catch (_) { }
  }
  if (state.writer) {
    try { state.writer.releaseLock(); } catch (_) { }
  }
  state.reader = null;
  state.writer = null;
  state.port = null;
  state.reading = false;
  while (state.txQueue.length > 0) state.txQueue.shift().resolve(false);
  state.txBusy = false;
}

async function readLoop() {
  const decoder = new TextDecoder();
  try {
    state.reader = state.port.readable.getReader();
    while (state.reading) {
      const { value, done } = await state.reader.read();
      if (done) break;
      if (value) processIncoming(decoder.decode(value, { stream: true }));
    }
  } catch (error) {
    if (state.reading) addLog(`[RX ERROR] ${error.message}`);
  } finally {
    if (state.reader) {
      try { state.reader.releaseLock(); } catch (_) { }
      state.reader = null;
    }
    if (state.reading) {
      cleanupPort();
      setConnected(false, "串口连接已关闭");
    }
  }
}

function processIncoming(chunk) {
  state.lineBuffer += chunk;
  const lines = state.lineBuffer.split(/\r?\n/);
  state.lineBuffer = lines.pop();
  lines.forEach(processLine);
}

function processLine(rawLine) {
  const line = rawLine.trim();
  if (!line) return;
  state.rxLines += 1;
  if (line.startsWith("{")) {
    try {
      const event = JSON.parse(line);
      if (event.type === "event") addEvent(event);
      else addLog(line);
    } catch (_) {
      state.invalidLines += 1;
      addLog(`[JSON ERROR] ${line}`);
    }
    updateMetrics();
    return;
  }
  const values = line.split(",").map((value) => Number.parseFloat(value.trim()));
  if (values.length === 4 && values.every(Number.isFinite) && values.every((value) => Math.abs(value) <= TELEMETRY_ABS_LIMIT_A)) {
    const [iqRef, iq, idRef, id] = values;
    state.samples.push({ time: performance.now(), iqRef, iq, idRef, id });
    state.samples = state.samples.slice(-MAX_SAMPLES);
    state.telemetryLines += 1;
    state.telemetryTimes.push(performance.now());
    updateReadout(iqRef, iq, idRef, id);
    renderChart();
  } else {
    processDeviceStatus(line);
    if (!line.startsWith("# UART RX IRQ READY")) addLog(line);
    if (!line.startsWith("#")) {
      state.invalidLines += 1;
      if (values.length === 4) elements.currentWarning.textContent = "已丢弃异常遥测";
    }
  }
  updateMetrics();
}

function setPidInputs(group, p, i, d) {
  const fieldset = document.querySelector(`fieldset[data-pid="${group}"]`);
  if (!fieldset) return;
  fieldset.querySelector('[data-gain="p"]').value = p;
  fieldset.querySelector('[data-gain="i"]').value = i;
  fieldset.querySelector('[data-gain="d"]').value = d;
}

function processDeviceStatus(line) {
  if (!line.startsWith("#")) return;
  const configMatch = line.match(/^# mode=(\S+) position=([-+.\d]+)deg speed=([-+.\d]+)rad\/s id=([-+.\d]+) iq=([-+.\d]+) max_speed=([-+.\d]+) max_torque=([-+.\d]+)/);
  if (configMatch) {
    const [, mode, position, speed, id, iq, maxSpeed, maxTorque] = configMatch;
    elements.mode.value = mode;
    elements.targetPosition.value = position;
    elements.targetSpeed.value = speed;
    elements.targetId.value = id;
    elements.targetIq.value = iq;
    elements.limitSpeed.value = maxSpeed;
    elements.limitTorque.value = maxTorque;
    return;
  }
  const pidMatch = line.match(/^# pid position=([-+.\d]+),([-+.\d]+),([-+.\d]+) speed=([-+.\d]+),([-+.\d]+),([-+.\d]+)/);
  if (pidMatch) {
    setPidInputs("position", pidMatch[1], pidMatch[2], pidMatch[3]);
    setPidInputs("speed", pidMatch[4], pidMatch[5], pidMatch[6]);
    return;
  }
  const currentPidMatch = line.match(/^# pid id=([-+.\d]+),([-+.\d]+),([-+.\d]+) iq=([-+.\d]+),([-+.\d]+),([-+.\d]+)/);
  if (currentPidMatch) {
    setPidInputs("id", currentPidMatch[1], currentPidMatch[2], currentPidMatch[3]);
    setPidInputs("iq", currentPidMatch[4], currentPidMatch[5], currentPidMatch[6]);
  }
}

function updateReadout(iqRef, iq, idRef, id) {
  elements.iqRef.textContent = `${iqRef.toFixed(3)} A`;
  elements.iqActual.textContent = `${iq.toFixed(3)} A`;
  elements.idRef.textContent = `${idRef.toFixed(3)} A`;
  elements.idActual.textContent = `${id.toFixed(3)} A`;
  const currentLimit = MAX_CURRENT_A * 0.95;
  const mismatch = Math.abs(iqRef - iq);
  if (Math.max(Math.abs(iq), Math.abs(id)) >= currentLimit) {
    elements.currentWarning.textContent = "接近最大电流";
    elements.currentWarning.className = "badge";
  } else if (mismatch > Math.max(0.08, Math.abs(iqRef) * 0.35)) {
    elements.currentWarning.textContent = "Iq 跟踪误差偏大";
    elements.currentWarning.className = "badge";
  } else {
    elements.currentWarning.textContent = "遥测正常";
    elements.currentWarning.className = "badge badge-muted";
  }
}

function renderChart() {
  const canvas = elements.telemetryChart;
  const context = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect();
  const pixelRatio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * pixelRatio));
  const height = Math.max(1, Math.round(rect.height * pixelRatio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  const cssWidth = rect.width;
  const cssHeight = rect.height;
  context.clearRect(0, 0, cssWidth, cssHeight);
  const padding = { left: 48, right: 16, top: 18, bottom: 28 };
  const plotWidth = cssWidth - padding.left - padding.right;
  const plotHeight = cssHeight - padding.top - padding.bottom;
  const visibleCount = Number(elements.sampleWindow.value);
  const samples = state.samples.slice(-visibleCount);
  if (samples.length === 0) {
    context.fillStyle = chartColors.label;
    context.font = "13px Segoe UI";
    context.textAlign = "center";
    context.fillText("等待 MCU 输出四列 CSV 遥测数据", cssWidth / 2, cssHeight / 2);
    return;
  }
  const allValues = samples.flatMap((sample) => [sample.iqRef, sample.iq, sample.idRef, sample.id]);
  const absoluteMax = Math.max(0.05, ...allValues.map(Math.abs));
  const range = Math.ceil(absoluteMax * 1.2 * 20) / 20;
  const min = -range;
  const max = range;
  const valueY = (value) => padding.top + (max - value) / (max - min) * plotHeight;
  const indexX = (index) => padding.left + (samples.length === 1 ? plotWidth / 2 : index / (samples.length - 1) * plotWidth);
  context.lineWidth = 1;
  context.strokeStyle = chartColors.grid;
  context.fillStyle = chartColors.label;
  context.font = "11px ui-monospace";
  context.textAlign = "right";
  for (let step = 0; step <= 4; step += 1) {
    const y = padding.top + (step / 4) * plotHeight;
    const value = max - (step / 4) * (max - min);
    context.beginPath(); context.moveTo(padding.left, y); context.lineTo(cssWidth - padding.right, y); context.stroke();
    context.fillText(`${value.toFixed(2)} A`, padding.left - 6, y + 4);
  }
  context.strokeStyle = chartColors.axis;
  context.beginPath(); context.moveTo(padding.left, valueY(0)); context.lineTo(cssWidth - padding.right, valueY(0)); context.stroke();
  const drawSeries = (key, color) => {
    context.strokeStyle = color;
    context.lineWidth = 1.7;
    context.beginPath();
    samples.forEach((sample, index) => {
      const x = indexX(index); const y = valueY(sample[key]);
      if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
    });
    context.stroke();
  };
  drawSeries("iqRef", chartColors.iqRef);
  drawSeries("iq", chartColors.iq);
  drawSeries("idRef", chartColors.idRef);
  drawSeries("id", chartColors.id);
  context.textAlign = "left";
  context.fillStyle = chartColors.label;
  context.fillText(`${samples.length} 点`, padding.left, cssHeight - 8);
}

function applyTargets() {
  try {
    const iq = finiteInput(elements.targetIq);
    const id = finiteInput(elements.targetId);
    const speed = finiteInput(elements.targetSpeed);
    const position = finiteInput(elements.targetPosition);
    if (Math.abs(iq) > 1 || Math.abs(id) > 1) throw new Error("Id/Iq 归一化值必须在 -1 到 1 之间。 ");
    send(`target id ${id}`);
    send(`target iq ${iq}`);
    send(`target speed ${speed}`);
    send(`target position ${position}`);
  } catch (error) { addLog(`[UI] ${error.message}`); }
}

async function applyPid(event) {
  if (state.operationBusy) return;
  setOperationBusy(true);
  try {
    const fieldset = event.currentTarget.closest("fieldset");
    const group = fieldset.dataset.pid;
    const gains = ["p", "i", "d"].map((name) => finiteInput(fieldset.querySelector(`[data-gain="${name}"]`)));
    if (gains.some((gain) => gain < 0)) throw new Error("PID 参数不能为负数。 ");
    if (elements.mode.value !== "null") {
      addLog("[UI] 在线修改 PID 会先停机，应用后请手动重新选择运行模式。 ");
      await send("mode null", true);
      elements.mode.value = "null";
    }
    await send(`pid ${group} ${gains[0]} ${gains[1]} ${gains[2]}`);
    await send("show");
    elements.currentWarning.textContent = "PID 已应用，电机保持停机";
    elements.currentWarning.className = "badge badge-muted";
  } catch (error) { addLog(`[UI] ${error.message}`); }
  finally { setOperationBusy(false); }
}

function bindEvents() {
  elements.connectButton.addEventListener("click", () => state.port ? disconnect() : connect());
  elements.emergencyStop.addEventListener("click", () => send("mode null", true));
  elements.applyMode.addEventListener("click", () => send(`mode ${elements.mode.value}`));
  elements.applyTargets.addEventListener("click", applyTargets);
  elements.applyLimits.addEventListener("click", () => {
    try {
      const torque = finiteInput(elements.limitTorque);
      const speed = finiteInput(elements.limitSpeed);
      if (torque < 0 || torque > 1 || speed < 0) throw new Error("限制参数超出允许范围。 ");
      send(`limit torque ${torque}`);
      send(`limit speed ${speed}`);
    } catch (error) { addLog(`[UI] ${error.message}`); }
  });
  elements.readConfig.addEventListener("click", () => send("show"));
  elements.sendManual.addEventListener("click", () => send(elements.manualCommand.value));
  elements.manualCommand.addEventListener("keydown", (event) => { if (event.key === "Enter") send(elements.manualCommand.value); });
  elements.pidApplyButtons.forEach((button) => button.addEventListener("click", applyPid));
  elements.clearLog.addEventListener("click", () => { elements.deviceLog.textContent = ""; });
  elements.clearChart.addEventListener("click", () => { state.samples = []; renderChart(); });
  elements.sampleWindow.addEventListener("change", renderChart);
  window.addEventListener("beforeunload", () => { if (state.writer) state.writer.write(new TextEncoder().encode("mode null\r\n")); });
  if ("serial" in navigator) navigator.serial.addEventListener("disconnect", () => { cleanupPort(); setConnected(false, "串口设备已移除"); addLog("[UI] 串口设备已移除。 "); });
  state.resizeObserver = new ResizeObserver(renderChart);
  state.resizeObserver.observe(document.querySelector(".chart-wrap"));
}

function initialize() {
  setConnected(false);
  bindEvents();
  renderChart();
  if (!("serial" in navigator)) addLog("[UI] 浏览器不支持 Web Serial。请使用最新版 Edge 或 Chrome。 ");
  addLog("[UI] 上位机已就绪。连接后会先自动发送 show。 ");
}

initialize();
