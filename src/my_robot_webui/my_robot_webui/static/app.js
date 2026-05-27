const state = {
  desktopUrl: "",
  stackEnsureRequested: false,
  pointRoute: [],
  lastConsumedReachedSeq: 0,
  lastStatusText: "",
  lastStatusSequence: 0,
};

const els = {
  stackBadge: document.getElementById("stackBadge"),
  desktopBadge: document.getElementById("desktopBadge"),
  cameraBadge: document.getElementById("cameraBadge"),
  currentStatus: document.getElementById("currentStatus"),
  statusHistory: document.getElementById("statusHistory"),
  stackLogs: document.getElementById("stackLogs"),
  commandMessage: document.getElementById("commandMessage"),
  parkingQueueStatus: document.getElementById("parkingQueueStatus"),
  cameraStream: document.getElementById("cameraStream"),
  cameraOverlay: document.getElementById("cameraOverlay"),
  topCameraStream: document.getElementById("topCameraStream"),
  desktopFrame: document.getElementById("desktopFrame"),
  desktopOverlay: document.getElementById("desktopOverlay"),
  customCommandForm: document.getElementById("customCommandForm"),
  customCommandInput: document.getElementById("customCommandInput"),
};

function setMessage(target, text, isError = false) {
  target.textContent = text || "";
  target.style.color = isError ? "#c44734" : "";
}

async function postJson(url, payload = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  return response.json();
}

function updateBadge(element, text, tone) {
  element.textContent = text;
  element.dataset.tone = tone || "neutral";
}

function setButtonPending(button, isPending) {
  if (!button) {
    return;
  }
  button.classList.toggle("is-pending", isPending);
}

function flashPressed(button) {
  if (!button) {
    return;
  }
  button.classList.remove("was-pressed");
  void button.offsetWidth;
  button.classList.add("was-pressed");
  window.setTimeout(() => {
    button.classList.remove("was-pressed");
  }, 900);
}

function setSelectedCommandButton(activeButton) {
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.classList.toggle("is-selected", button === activeButton);
  });
}

function extractPointLabel(command) {
  const match = /^park at ([ABCD])$/i.exec(command || "");
  return match ? match[1].toUpperCase() : "";
}

function extractReachedPoint(statusText) {
  const match = /\bREACHED_([ABCD])\b/i.exec(statusText || "");
  return match ? match[1].toUpperCase() : "";
}

function buildReachedStatusKey(command) {
  const pointLabel = extractPointLabel(command);
  return pointLabel ? `REACHED_${pointLabel}` : "";
}

function isPointQueueButton(button) {
  return button?.dataset.queueGroup === "parking-point";
}

function updatePointQueueVisuals() {
  const activeCommand = state.pointRoute[0] || "";
  const queuedCommands = new Set(state.pointRoute.slice(1));

  document.querySelectorAll('[data-queue-group="parking-point"]').forEach((button) => {
    const command = button.dataset.command;
    const isActive = command === activeCommand;
    const isQueued = queuedCommands.has(command);
    button.classList.toggle("is-active-queue", isActive);
    button.classList.toggle("is-queued", !isActive && isQueued);
  });

  const queueLabels = state.pointRoute.map((command, index) => {
    const label = extractPointLabel(command);
    if (!label) {
      return "";
    }
    return index === 0 ? `${label} (running)` : label;
  }).filter(Boolean);

  if (els.parkingQueueStatus) {
    els.parkingQueueStatus.textContent = queueLabels.length
      ? `Point queue: ${queueLabels.join(" -> ")}`
      : "Point queue: empty";
  }
}

function clearPointQueueState() {
  state.pointRoute = [];
  state.lastConsumedReachedSeq = state.lastStatusSequence;
  updatePointQueueVisuals();
}

function buildPointRouteCommand(route) {
  if (!Array.isArray(route) || !route.length) {
    return "";
  }

  if (route.length === 1) {
    return route[0];
  }

  const labels = route
    .map((command) => extractPointLabel(command))
    .filter(Boolean);

  if (!labels.length) {
    return "";
  }

  return `park route ${labels.join(" ")}`;
}

async function syncPointRouteToRobot() {
  const routeCommand = buildPointRouteCommand(state.pointRoute);
  if (!routeCommand) {
    updatePointQueueVisuals();
    return;
  }

  state.lastConsumedReachedSeq = state.lastStatusSequence;

  try {
    const result = await postJson("/api/command", { command: routeCommand });
    setMessage(els.commandMessage, result.message, !result.ok);
    if (!result.ok) {
      return false;
    }
    refreshState();
    return true;
  } catch (error) {
    setMessage(els.commandMessage, "Failed to update point queue.", true);
    return false;
  }
}

function completeQueuedPoints(statusEvents) {
  if (!state.pointRoute.length || !Array.isArray(statusEvents) || !statusEvents.length) {
    return;
  }

  const newEvents = [...statusEvents]
    .filter((event) => {
      if (!event || typeof event !== "object") {
        return false;
      }
      return Number(event.seq || 0) > state.lastConsumedReachedSeq;
    })
    .sort((left, right) => Number(left.seq || 0) - Number(right.seq || 0));

  if (!newEvents.length) {
    return;
  }

  for (const event of newEvents) {
    const seq = Number(event.seq || 0);
    const text = String(event.text || "");
    const reachedPoint = extractReachedPoint(text);
    const activePoint = extractPointLabel(state.pointRoute[0] || "");
    state.lastConsumedReachedSeq = seq;
    if (!reachedPoint || !activePoint || reachedPoint !== activePoint) {
      continue;
    }
    state.pointRoute.shift();
  }

  updatePointQueueVisuals();
}

function renderLogs(lines) {
  return Array.isArray(lines) && lines.length ? lines.join("\n") : "No logs yet.";
}

function renderStatusHistory(lines) {
  return Array.isArray(lines) && lines.length ? lines.join("\n") : "No mission status received yet.";
}

function formatMode(mode) {
  if (mode === "road") {
    return "Road Mission";
  }
  return "Parking Mission";
}

function renderDesktop(stateData) {
  const desktop = stateData.desktop;
  const ros = stateData.ros;
  const ready = desktop.novnc.running;
  const installed = desktop.x11vnc_installed && desktop.novnc_installed;
  state.desktopUrl = desktop.novnc_url;

  if (ros.top_camera_available) {
    if (!els.topCameraStream.src || !els.topCameraStream.src.endsWith(stateData.top_camera_stream_url)) {
      els.topCameraStream.src = stateData.top_camera_stream_url;
    }
    els.topCameraStream.style.display = "block";
    els.desktopFrame.style.display = "none";
    els.desktopOverlay.style.display = "none";
    updateBadge(els.desktopBadge, "Gazebo: top view live", "good");
    return;
  }

  els.topCameraStream.removeAttribute("src");
  els.topCameraStream.style.display = "none";

  if (ready) {
    if (els.desktopFrame.src !== desktop.novnc_url) {
      els.desktopFrame.src = desktop.novnc_url;
    }
    els.desktopFrame.style.display = "block";
    els.desktopOverlay.style.display = "none";
    updateBadge(els.desktopBadge, "Gazebo: desktop stream live", "good");
    return;
  }

  els.desktopFrame.removeAttribute("src");
  els.desktopFrame.style.display = "none";
  els.desktopOverlay.style.display = "flex";

  if (!desktop.display_available) {
    els.desktopOverlay.textContent = "Waiting for the overhead Gazebo camera. If you want browser mirroring as a fallback, start the dashboard from a session with DISPLAY set.";
    updateBadge(els.desktopBadge, "Gazebo: top view offline", "bad");
    return;
  }

  if (!installed) {
    const missing = [];
    if (!desktop.x11vnc_installed) missing.push("x11vnc");
    if (!desktop.novnc_installed) missing.push("novnc_proxy");
    els.desktopOverlay.textContent = `Waiting for /top_view_camera/image_raw. Desktop fallback also needs: ${missing.join(", ")}.`;
    updateBadge(els.desktopBadge, "Gazebo: top view offline", "bad");
    return;
  }

  if ((desktop.session_type || "").toLowerCase() === "wayland") {
    els.desktopOverlay.textContent = "Waiting for /top_view_camera/image_raw. Desktop fallback is limited in Wayland sessions.";
    updateBadge(els.desktopBadge, "Gazebo: top view offline", "bad");
    return;
  }

  els.desktopOverlay.textContent = "Waiting for the overhead Gazebo camera. Desktop fallback will appear when noVNC starts correctly.";
  updateBadge(els.desktopBadge, "Gazebo: waiting for top view", "neutral");
}

function renderCamera(stateData) {
  const ros = stateData.ros;
  if (ros.camera_available) {
    if (!els.cameraStream.src || !els.cameraStream.src.endsWith(stateData.camera_stream_url)) {
      els.cameraStream.src = stateData.camera_stream_url;
    }
    els.cameraOverlay.style.display = "none";
    updateBadge(els.cameraBadge, "Camera: live", "good");
  } else {
    els.cameraOverlay.style.display = "flex";
    els.cameraOverlay.textContent = "Waiting for /camera/image_raw frames...";
    updateBadge(els.cameraBadge, "Camera: offline", "bad");
  }
}

function renderState(data) {
  const stack = data.stack;
  const ros = data.ros;
  const autoLaunch = data.auto_launch || {};
  const profileLabel = formatMode(autoLaunch.mode || stack.mode || "parking");

  updateBadge(
    els.stackBadge,
    stack.running ? `Stack: ${profileLabel}` : "Stack: offline",
    stack.running ? "good" : "bad"
  );

  const statusText = ros.last_status || "Waiting for mission status...";
  els.currentStatus.textContent = statusText;
  els.statusHistory.textContent = renderStatusHistory(ros.status_history);
  els.stackLogs.textContent = renderLogs(stack.logs);
  renderCamera(data);
  renderDesktop(data);
  completeQueuedPoints(ros.status_events);
  state.lastStatusText = statusText;
  state.lastStatusSequence = Number(ros.status_sequence || 0);
  updatePointQueueVisuals();
}

async function refreshState() {
  try {
    const response = await fetch("/api/state");
    const data = await response.json();
    renderState(data);
    ensureStackStarted(data);
  } catch (error) {
    updateBadge(els.stackBadge, "Stack: dashboard disconnected", "bad");
    updateBadge(els.desktopBadge, "Gazebo: dashboard disconnected", "bad");
    updateBadge(els.cameraBadge, "Camera: dashboard disconnected", "bad");
    els.cameraOverlay.style.display = "flex";
    els.desktopOverlay.style.display = "flex";
    els.cameraOverlay.textContent = "Dashboard backend is unavailable.";
    els.desktopOverlay.textContent = "Dashboard backend is unavailable.";
    els.topCameraStream.style.display = "none";
    els.desktopFrame.style.display = "none";
  }
}

async function ensureStackStarted(data) {
  const autoLaunch = data.auto_launch || {};
  if (state.stackEnsureRequested || data.stack?.running) {
    return;
  }

  state.stackEnsureRequested = true;
  const mode = autoLaunch.mode || "parking";

  try {
    const result = await postJson("/api/stack/start", { mode });
    if (!result.ok && !result.message.includes("already running")) {
      console.warn(result.message);
    }
  } catch (error) {
    state.stackEnsureRequested = false;
    console.warn("Automatic mission launch request failed.");
    return;
  }

  refreshState();
}

document.querySelectorAll("[data-command]").forEach((button) => {
  button.addEventListener("click", async () => {
    const command = button.dataset.command;
    if (isPointQueueButton(button)) {
      if (state.pointRoute.includes(command)) {
        setMessage(
          els.commandMessage,
          `Point ${extractPointLabel(command)} is already in the queue.`
        );
        flashPressed(button);
        updatePointQueueVisuals();
        return;
      }

      state.pointRoute.push(command);
      flashPressed(button);
      updatePointQueueVisuals();
      setMessage(
        els.commandMessage,
        `Queued point ${extractPointLabel(command)}. It will run in order.`
      );
      const synced = await syncPointRouteToRobot();
      if (!synced) {
        state.pointRoute = state.pointRoute.filter((item) => item !== command);
        updatePointQueueVisuals();
      }
      return;
    }

    setButtonPending(button, true);
    try {
      clearPointQueueState();
      const result = await postJson("/api/command", { command });
      setMessage(els.commandMessage, result.message, !result.ok);
      flashPressed(button);
      if (result.ok) {
        setSelectedCommandButton(button);
      }
      refreshState();
    } finally {
      setButtonPending(button, false);
    }
  });
});

els.customCommandForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const command = els.customCommandInput.value.trim();
  if (!command) {
    setMessage(els.commandMessage, "Type a command first.", true);
    return;
  }
  const submitButton = els.customCommandForm.querySelector('button[type="submit"]');
  setButtonPending(submitButton, true);
  try {
    clearPointQueueState();
    const result = await postJson("/api/command", { command });
    setMessage(els.commandMessage, result.message, !result.ok);
    flashPressed(submitButton);
    if (result.ok) {
      els.customCommandInput.value = "";
      setSelectedCommandButton(null);
    }
    refreshState();
  } finally {
    setButtonPending(submitButton, false);
  }
});

updatePointQueueVisuals();
refreshState();
setInterval(refreshState, 2000);
