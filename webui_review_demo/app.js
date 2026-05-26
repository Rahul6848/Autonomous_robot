const statusHistory = [
  'PARKED_AT_B - demo mission complete.',
  'VISION_PARK - slot alignment locked.',
  'TURN_TO_SLOT - rotating into selected parking slot.',
  'SCAN_RIGHT - slot on right side appears clear.',
  'APPROACHING_B - driving toward target point B.',
  'IDLE - awaiting high-level command.'
];

const stackLogs = [
  '$ ros2 launch my_robot_bringup parking_mission.launch.xml',
  '[dashboard] review-demo mode enabled',
  '[dashboard] live ROS control disabled',
  '[stack] robot_state_publisher started',
  '[stack] gazebo_ros launched with parking world',
  '[stack] parking_mission_controller ready',
  '[stream] camera panel switched to preview asset',
  '[stream] desktop panel switched to static mock',
];

const commandResponses = {
  resume: 'Demo action: mission resumed from paused state.',
  'go check B': 'Demo action: route playback switched to road mission preview.',
  return: 'Demo action: return-to-origin sequence previewed.',
  stop: 'Demo action: emergency stop state highlighted.',
  'park at A': 'Demo action: parking target changed to A.',
  'park at B': 'Demo action: parking target changed to B.',
  'park at C': 'Demo action: parking target changed to C.',
  'park at D': 'Demo action: parking target changed to D.',
};

const currentStatusByCommand = {
  resume: 'RESUMED - demo playback continued.',
  'go check B': 'GOING_TO_B - demo road mission in progress.',
  return: 'RETURNING_TO_IDLE - demo return sequence in progress.',
  stop: 'STOPPED - demo safety halt active.',
  'park at A': 'APPROACHING_A - demo target selected.',
  'park at B': 'APPROACHING_B - demo target selected.',
  'park at C': 'APPROACHING_C - demo target selected.',
  'park at D': 'APPROACHING_D - demo target selected.',
};

const els = {
  stackBadge: document.getElementById('stackBadge'),
  desktopBadge: document.getElementById('desktopBadge'),
  cameraBadge: document.getElementById('cameraBadge'),
  currentStatus: document.getElementById('currentStatus'),
  statusHistory: document.getElementById('statusHistory'),
  stackLogs: document.getElementById('stackLogs'),
  commandMessage: document.getElementById('commandMessage'),
  customCommandForm: document.getElementById('customCommandForm'),
  customCommandInput: document.getElementById('customCommandInput'),
  cameraStream: document.getElementById('cameraStream'),
  desktopPreview: document.getElementById('desktopPreview'),
};

function svgDataUrl(markup) {
  return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(markup)}`;
}

function buildCameraPreview() {
  return svgDataUrl(`
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 640">
      <defs>
        <linearGradient id="sky" x1="0" x2="0" y1="0" y2="1">
          <stop offset="0%" stop-color="#7fb7e7"/>
          <stop offset="100%" stop-color="#d8edf9"/>
        </linearGradient>
        <linearGradient id="road" x1="0" x2="0" y1="0" y2="1">
          <stop offset="0%" stop-color="#3a4147"/>
          <stop offset="100%" stop-color="#252c31"/>
        </linearGradient>
      </defs>
      <rect width="960" height="640" fill="url(#sky)"/>
      <rect x="0" y="350" width="960" height="290" fill="#87a367"/>
      <polygon points="360,640 600,640 530,260 430,260" fill="url(#road)"/>
      <line x1="480" y1="640" x2="480" y2="270" stroke="#ffd868" stroke-width="12" stroke-dasharray="26 24"/>
      <rect x="90" y="170" width="120" height="120" rx="16" fill="#f5fafc" opacity="0.9"/>
      <path d="M120 230h60" stroke="#2d6ba4" stroke-width="18" stroke-linecap="round"/>
      <path d="M150 200v60" stroke="#2d6ba4" stroke-width="18" stroke-linecap="round"/>
      <text x="54" y="72" fill="#16304a" font-size="34" font-family="Arial, sans-serif" font-weight="700">Robot Camera Preview</text>
      <text x="54" y="112" fill="#214968" font-size="24" font-family="Arial, sans-serif">Static review asset for UI demonstration</text>
    </svg>
  `);
}

function buildDesktopPreview() {
  return svgDataUrl(`
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 640">
      <defs>
        <linearGradient id="bg" x1="0" x2="1" y1="0" y2="1">
          <stop offset="0%" stop-color="#16304b"/>
          <stop offset="100%" stop-color="#0d1e2f"/>
        </linearGradient>
      </defs>
      <rect width="960" height="640" fill="url(#bg)"/>
      <circle cx="782" cy="106" r="84" fill="#23415d" opacity="0.36"/>
      <circle cx="162" cy="502" r="110" fill="#244566" opacity="0.24"/>
      <text x="42" y="62" fill="#dcecfb" font-size="34" font-family="Arial, sans-serif" font-weight="700">Embedded Gazebo + RViz Preview</text>
      <text x="42" y="100" fill="#9bbcd8" font-size="22" font-family="Arial, sans-serif">Static panel shown in review mode</text>
    </svg>
  `);
}

function renderStatusHistory() {
  els.statusHistory.textContent = statusHistory.join('\n');
}

function renderStackLogs() {
  els.stackLogs.textContent = stackLogs.join('\n');
}

function updateSelection(activeButton) {
  document.querySelectorAll('[data-command]').forEach((button) => {
    button.classList.toggle('is-selected', button === activeButton);
  });
}

function setCommandFeedback(message, isError = false) {
  els.commandMessage.textContent = message;
  els.commandMessage.style.color = isError ? '#c44734' : '';
}

function pushStatus(line) {
  statusHistory.unshift(line);
  if (statusHistory.length > 8) {
    statusHistory.pop();
  }
  els.currentStatus.textContent = line;
  renderStatusHistory();
}

function pushLog(line) {
  stackLogs.unshift(line);
  if (stackLogs.length > 10) {
    stackLogs.pop();
  }
  renderStackLogs();
}

function runDemoCommand(command, sourceLabel) {
  const normalized = command.trim();
  if (!normalized) {
    setCommandFeedback('Type a command first.', true);
    return;
  }

  const result = commandResponses[normalized] || `Demo action: "${normalized}" captured in review mode only.`;
  const status = currentStatusByCommand[normalized] || `DEMO_COMMAND - preview accepted "${normalized}".`;
  pushStatus(status);
  pushLog(`[demo] ${sourceLabel} -> ${normalized}`);
  setCommandFeedback(result);
}

document.querySelectorAll('[data-command]').forEach((button) => {
  button.addEventListener('click', () => {
    updateSelection(button);
    runDemoCommand(button.dataset.command || '', 'button');
  });
});

els.customCommandForm.addEventListener('submit', (event) => {
  event.preventDefault();
  updateSelection(null);
  runDemoCommand(els.customCommandInput.value, 'custom');
  els.customCommandInput.value = '';
});

els.cameraStream.src = buildCameraPreview();
els.desktopPreview.src = buildDesktopPreview();
renderStatusHistory();
renderStackLogs();
