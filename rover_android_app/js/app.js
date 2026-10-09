/**
 * Rover Android App - Main Application Orchestrator
 * Integrates ROSLib with CanvasMap, VirtualJoystick, Waypoint Missions,
 * and Android Touch Controls.
 */

// Global State
const appState = {
  ros: null,
  connected: false,
  rosBridgeIp: localStorage.getItem('rover_ws_ip') || window.location.hostname || '192.168.1.54',
  rosBridgePort: localStorage.getItem('rover_ws_port') || '9090',
  maxLinearSpeed: parseFloat(localStorage.getItem('rover_max_speed') || '0.12'),
  showScan: true,
  waypoints: JSON.parse(localStorage.getItem('rover_waypoints') || '[]'),
  isMissionRunning: false,
  missionIndex: 0,
};

// UI Elements
const el = {
  connDot: document.getElementById('connStatusDot'),
  connText: document.getElementById('connStatusText'),
  teleX: document.getElementById('teleX'),
  teleY: document.getElementById('teleY'),
  teleYaw: document.getElementById('teleYaw'),
  modeHint: document.getElementById('modeHint'),
  modeHintText: document.getElementById('modeHintText'),
  btnRecenter: document.getElementById('btnRecenter'),
  btnSettings: document.getElementById('btnSettings'),
  btnEStop: document.getElementById('btnEStop'),
  btnZoomIn: document.getElementById('btnZoomIn'),
  btnZoomOut: document.getElementById('btnZoomOut'),
  btnResetView: document.getElementById('btnResetView'),
  joystickZone: document.getElementById('joystickZone'),
  drawer: document.getElementById('waypointDrawer'),
  btnCloseDrawer: document.getElementById('btnCloseDrawer'),
  wpList: document.getElementById('waypointList'),
  wpBadge: document.getElementById('wpBadge'),
  wpCountLabel: document.getElementById('wpCountLabel'),
  btnAddCurrentPose: document.getElementById('btnAddCurrentPose'),
  btnClearWaypoints: document.getElementById('btnClearWaypoints'),
  btnStartMission: document.getElementById('btnStartMission'),
  btnCancelMission: document.getElementById('btnCancelMission'),
  settingsModal: document.getElementById('settingsModal'),
  btnCloseSettings: document.getElementById('btnCloseSettings'),
  btnSaveConnect: document.getElementById('btnSaveConnect'),
  wsIpInput: document.getElementById('wsIpInput'),
  wsPortInput: document.getElementById('wsPortInput'),
  maxSpeedSlider: document.getElementById('maxSpeedSlider'),
  maxSpeedVal: document.getElementById('maxSpeedVal'),
  chkShowScan: document.getElementById('chkShowScan'),
  toolButtons: document.querySelectorAll('.tool-btn'),
};

// Subscriptions & Publishers
let topics = {
  mapSub: null,
  poseSub: null,
  pathSub: null,
  scanSub: null,
  initialPosePub: null,
  goalPosePub: null,
  cmdVelPub: null,
};

// Initialize Canvas Map & Joystick
const canvasMap = new CanvasMap('mapCanvas');
let joystick = null;

// --- Helper: Yaw to Quaternion ---
function yawToQuaternion(yaw) {
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  return { x: 0.0, y: 0.0, z: sy, w: cy };
}

function quaternionToYaw(q) {
  const siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny_cosp, cosy_cosp);
}

// ==============================================================================
// 1. ROS Connection Management
// ==============================================================================
function connectROS() {
  const url = `ws://${appState.rosBridgeIp}:${appState.rosBridgePort}`;
  console.log(`Connecting to ROS Bridge at ${url}...`);
  el.connText.textContent = `Connecting to ${appState.rosBridgeIp}...`;
  el.connDot.className = 'status-indicator';

  if (appState.ros) {
    try { appState.ros.close(); } catch (e) {}
  }

  appState.ros = new ROSLIB.Ros({ url: url });

  appState.ros.on('connection', () => {
    console.log('Connected to ROS Bridge!');
    appState.connected = true;
    el.connDot.className = 'status-indicator connected';
    el.connText.textContent = 'Rover Online (ROS 2)';
    setupTopics();
  });

  appState.ros.on('error', (err) => {
    console.warn('ROS Bridge connection error:', err);
    appState.connected = false;
    el.connDot.className = 'status-indicator';
    el.connText.textContent = 'Connection Error';
  });

  appState.ros.on('close', () => {
    console.log('ROS Bridge connection closed.');
    appState.connected = false;
    el.connDot.className = 'status-indicator';
    el.connText.textContent = 'Disconnected (Retrying...)';
    setTimeout(connectROS, 5000);
  });
}

// ==============================================================================
// 2. ROS Topics Setup
// ==============================================================================
function setupTopics() {
  const ros = appState.ros;

  // 1. /map (OccupancyGrid)
  topics.mapSub = new ROSLIB.Topic({
    ros: ros,
    name: '/map',
    messageType: 'nav_msgs/msg/OccupancyGrid',
    compression: 'png'
  });
  topics.mapSub.subscribe((msg) => {
    canvasMap.updateMap(msg);
  });

  // 2. /amcl_pose (Robot Pose)
  topics.poseSub = new ROSLIB.Topic({
    ros: ros,
    name: '/amcl_pose',
    messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped'
  });
  topics.poseSub.subscribe((msg) => {
    const pos = msg.pose.pose.position;
    const ori = msg.pose.pose.orientation;
    const yaw = quaternionToYaw(ori);

    canvasMap.roverPose = { x: pos.x, y: pos.y, yaw: yaw, valid: true };
    el.teleX.textContent = pos.x.toFixed(2);
    el.teleY.textContent = pos.y.toFixed(2);
    el.teleYaw.textContent = `${Math.round((yaw * 180) / Math.PI)}°`;
  });

  // 3. /plan (Global Path)
  topics.pathSub = new ROSLIB.Topic({
    ros: ros,
    name: '/plan',
    messageType: 'nav_msgs/msg/Path'
  });
  topics.pathSub.subscribe((msg) => {
    canvasMap.updateGlobalPath(msg);
  });

  // 4. /scan (LaserScan)
  topics.scanSub = new ROSLIB.Topic({
    ros: ros,
    name: '/scan',
    messageType: 'sensor_msgs/msg/LaserScan',
    throttle_rate: 200 // throttle to 5 Hz on mobile
  });
  topics.scanSub.subscribe((msg) => {
    if (appState.showScan) {
      canvasMap.updateLaserScan(msg);
    }
  });

  // 5. Publishers
  topics.initialPosePub = new ROSLIB.Topic({
    ros: ros,
    name: '/initialpose',
    messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped'
  });

  topics.goalPosePub = new ROSLIB.Topic({
    ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/msg/PoseStamped'
  });

  topics.cmdVelPub = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel_smoothed',
    messageType: 'geometry_msgs/msg/Twist'
  });
}

// ==============================================================================
// 3. Publishing Commands (Pose, Goal, Teleop)
// ==============================================================================
function publishInitialPose(x, y, yaw) {
  if (!appState.connected || !topics.initialPosePub) return;
  const q = yawToQuaternion(yaw);
  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      pose: {
        position: { x: x, y: y, z: 0.0 },
        orientation: q
      },
      covariance: [
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.068
      ]
    }
  });
  topics.initialPosePub.publish(msg);
  showToast(`🎯 Initial Pose published: (${x.toFixed(2)}, ${y.toFixed(2)})`);
  if (navigator.vibrate) navigator.vibrate(50);
}

function publishNavGoal(x, y, yaw) {
  if (!appState.connected || !topics.goalPosePub) return;
  const q = yawToQuaternion(yaw);
  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      position: { x: x, y: y, z: 0.0 },
      orientation: q
    }
  });
  topics.goalPosePub.publish(msg);
  showToast(`📍 Nav Goal sent: (${x.toFixed(2)}, ${y.toFixed(2)})`);
  if (navigator.vibrate) navigator.vibrate(50);
}

function publishCmdVel(linear, angular) {
  if (!appState.connected || !topics.cmdVelPub) return;
  const twist = new ROSLIB.Message({
    linear: { x: linear, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: angular }
  });
  topics.cmdVelPub.publish(twist);
}

function emergencyStop() {
  publishCmdVel(0, 0);
  if (appState.isMissionRunning) {
    cancelMission();
  }
  showToast('🛑 EMERGENCY STOP TRIGGERED');
  if (navigator.vibrate) navigator.vibrate([100, 50, 100]);
}

// ==============================================================================
// 4. Interactive Tools & Gestures
// ==============================================================================
canvasMap.onPoseConfirmed = (x, y, yaw, tool) => {
  if (tool === 'pose_est') {
    publishInitialPose(x, y, yaw);
    setTool('pan');
  } else if (tool === 'nav_goal') {
    publishNavGoal(x, y, yaw);
    setTool('pan');
  }
};

canvasMap.onWaypointAdded = (x, y, yaw) => {
  addWaypoint(`Point_${appState.waypoints.length + 1}`, x, y, yaw);
};

function setTool(toolName) {
  canvasMap.activeTool = toolName;
  el.toolButtons.forEach(btn => {
    btn.classList.toggle('active', btn.dataset.tool === toolName);
  });

  if (toolName === 'pose_est') {
    el.modeHintText.textContent = 'Touch and drag arrow to set 2D Initial Pose';
    el.modeHint.classList.remove('hidden');
  } else if (toolName === 'nav_goal') {
    el.modeHintText.textContent = 'Touch and drag arrow to set 2D Nav Goal';
    el.modeHint.classList.remove('hidden');
  } else if (toolName === 'waypoints') {
    el.modeHintText.textContent = 'Tap on map to add waypoints';
    el.modeHint.classList.remove('hidden');
  } else {
    el.modeHint.classList.add('hidden');
  }
}

// ==============================================================================
// 5. Waypoint Mission Management
// ==============================================================================
function addWaypoint(name, x, y, yaw = 0.0, waitSec = 3.0) {
  const wp = {
    name: name,
    x: parseFloat(x.toFixed(3)),
    y: parseFloat(y.toFixed(3)),
    yaw: parseFloat(yaw.toFixed(3)),
    wait: waitSec
  };
  appState.waypoints.push(wp);
  saveWaypoints();
  renderWaypoints();
  showToast(`Added Waypoint: ${name}`);
}

function removeWaypoint(index) {
  appState.waypoints.splice(index, 1);
  saveWaypoints();
  renderWaypoints();
}

function saveWaypoints() {
  localStorage.setItem('rover_waypoints', JSON.stringify(appState.waypoints));
  canvasMap.waypoints = appState.waypoints;
  el.wpBadge.textContent = appState.waypoints.length;
  el.wpCountLabel.textContent = `${appState.waypoints.length} locations`;
}

function renderWaypoints() {
  const list = el.wpList;
  list.innerHTML = '';

  if (appState.waypoints.length === 0) {
    list.innerHTML = `
      <div class="empty-state">
        <p>No waypoints added yet.</p>
        <small>Tap the map in "Waypoints" mode or click "Current Pose" above.</small>
      </div>`;
    return;
  }

  appState.waypoints.forEach((wp, idx) => {
    const item = document.createElement('div');
    item.className = 'wp-item';
    item.innerHTML = `
      <div class="wp-badge-num">${idx + 1}</div>
      <div class="wp-details">
        <div class="wp-name">${wp.name}</div>
        <div class="wp-coords">x: ${wp.x}m, y: ${wp.y}m | wait: ${wp.wait}s</div>
      </div>
      <button class="wp-del-btn" title="Remove">&times;</button>
    `;
    item.querySelector('.wp-del-btn').addEventListener('click', () => removeWaypoint(idx));
    list.appendChild(item);
  });
}

async function startMission() {
  if (appState.waypoints.length === 0) {
    showToast('Add waypoints before starting mission');
    return;
  }

  appState.isMissionRunning = true;
  appState.missionIndex = 0;
  el.btnStartMission.classList.add('hidden');
  el.btnCancelMission.classList.remove('hidden');
  el.drawer.classList.add('hidden');

  showToast(`Starting mission with ${appState.waypoints.length} waypoints!`);

  for (let i = 0; i < appState.waypoints.length; i++) {
    if (!appState.isMissionRunning) break;
    appState.missionIndex = i;
    const wp = appState.waypoints[i];

    showToast(`Navigating to Waypoint [${i + 1}/${appState.waypoints.length}]: ${wp.name}`);
    publishNavGoal(wp.x, wp.y, wp.yaw);

    // Wait until arrived or cancelled
    await waitForArrival(wp.x, wp.y);

    if (!appState.isMissionRunning) break;

    // Execute pause task
    if (wp.wait > 0) {
      showToast(`Arrived at '${wp.name}'. Pausing for ${wp.wait}s...`);
      await new Promise(r => setTimeout(r, wp.wait * 1000));
    }
  }

  if (appState.isMissionRunning) {
    showToast('🎉 Waypoint Mission Completed Successfully!');
  }
  cancelMission();
}

function cancelMission() {
  appState.isMissionRunning = false;
  el.btnStartMission.classList.remove('hidden');
  el.btnCancelMission.classList.add('hidden');
}

function waitForArrival(targetX, targetY, timeoutSec = 90) {
  return new Promise((resolve) => {
    const startTime = Date.now();
    const interval = setInterval(() => {
      if (!appState.isMissionRunning) {
        clearInterval(interval);
        resolve(false);
        return;
      }

      if (canvasMap.roverPose.valid) {
        const dist = Math.hypot(canvasMap.roverPose.x - targetX, canvasMap.roverPose.y - targetY);
        if (dist < 0.35) { // within 35 cm tolerance
          clearInterval(interval);
          resolve(true);
          return;
        }
      }

      if ((Date.now() - startTime) / 1000 > timeoutSec) {
        showToast('Waypoint navigation timeout reached');
        clearInterval(interval);
        resolve(false);
      }
    }, 500);
  });
}

// ==============================================================================
// 6. UI Event Listeners
// ==============================================================================
function showToast(msg) {
  let toast = document.getElementById('toast');
  if (!toast) {
    toast = document.createElement('div');
    toast.id = 'toast';
    toast.style.cssText = `
      position: absolute; bottom: 80px; left: 50%; transform: translateX(-50%);
      background: rgba(15, 23, 42, 0.95); border: 1px solid var(--border-accent);
      color: #fff; padding: 10px 20px; border-radius: 999px; font-size: 12px;
      font-weight: bold; z-index: 200; box-shadow: var(--shadow-glow-cyan);
      pointer-events: none; transition: opacity 0.3s ease; text-align: center;
    `;
    document.body.appendChild(toast);
  }
  toast.textContent = msg;
  toast.style.opacity = '1';
  clearTimeout(toast._timeout);
  toast._timeout = setTimeout(() => { toast.style.opacity = '0'; }, 3000);
}

function attachUIEvents() {
  // Toolbar buttons
  el.toolButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      const tool = btn.dataset.tool;
      if (tool === 'waypoints') {
        el.drawer.classList.toggle('hidden');
        setTool('waypoint');
      } else if (tool === 'joystick_toggle') {
        el.joystickZone.classList.toggle('hidden');
        btn.classList.toggle('active', !el.joystickZone.classList.contains('hidden'));
      } else {
        setTool(tool);
      }
    });
  });

  // Zoom and Recenter buttons
  el.btnRecenter.addEventListener('click', () => canvasMap.recenter());
  el.btnZoomIn.addEventListener('click', () => canvasMap.setZoom(1.25));
  el.btnZoomOut.addEventListener('click', () => canvasMap.setZoom(0.8));
  el.btnResetView.addEventListener('click', () => {
    canvasMap.zoom = 50.0;
    canvasMap.recenter(0, 0);
  });

  // Emergency Stop
  el.btnEStop.addEventListener('click', emergencyStop);

  // Drawer
  el.btnCloseDrawer.addEventListener('click', () => el.drawer.classList.add('hidden'));
  el.btnAddCurrentPose.addEventListener('click', () => {
    if (!canvasMap.roverPose.valid) {
      showToast('Rover position not known yet');
      return;
    }
    addWaypoint(`Pose_${appState.waypoints.length + 1}`, canvasMap.roverPose.x, canvasMap.roverPose.y, canvasMap.roverPose.yaw);
  });
  el.btnClearWaypoints.addEventListener('click', () => {
    if (confirm('Clear all waypoints?')) {
      appState.waypoints = [];
      saveWaypoints();
      renderWaypoints();
    }
  });
  el.btnStartMission.addEventListener('click', startMission);
  el.btnCancelMission.addEventListener('click', cancelMission);

  // Settings Modal
  el.btnSettings.addEventListener('click', () => {
    el.wsIpInput.value = appState.rosBridgeIp;
    el.wsPortInput.value = appState.rosBridgePort;
    el.maxSpeedSlider.value = appState.maxLinearSpeed;
    el.maxSpeedVal.textContent = appState.maxLinearSpeed.toFixed(2);
    el.settingsModal.classList.remove('hidden');
  });
  el.btnCloseSettings.addEventListener('click', () => el.settingsModal.classList.add('hidden'));
  el.maxSpeedSlider.addEventListener('input', (e) => {
    el.maxSpeedVal.textContent = parseFloat(e.target.value).toFixed(2);
  });
  el.btnSaveConnect.addEventListener('click', () => {
    appState.rosBridgeIp = el.wsIpInput.value.trim() || '192.168.1.54';
    appState.rosBridgePort = el.wsPortInput.value.trim() || '9090';
    appState.maxLinearSpeed = parseFloat(el.maxSpeedSlider.value);
    appState.showScan = el.chkShowScan.checked;

    localStorage.setItem('rover_ws_ip', appState.rosBridgeIp);
    localStorage.setItem('rover_ws_port', appState.rosBridgePort);
    localStorage.setItem('rover_max_speed', appState.maxLinearSpeed.toString());

    if (joystick) joystick.setMaxLinearSpeed(appState.maxLinearSpeed);
    el.settingsModal.classList.add('hidden');
    connectROS();
  });
}

// ==============================================================================
// 7. App Initialization
// ==============================================================================
window.addEventListener('DOMContentLoaded', () => {
  attachUIEvents();
  renderWaypoints();
  saveWaypoints();

  // Initialize Touch Joystick
  joystick = new VirtualJoystick('joystickBase', 'joystickStick', (linear, angular) => {
    publishCmdVel(linear, angular);
  });
  joystick.setMaxLinearSpeed(appState.maxLinearSpeed);

  // Start ROS connection
  connectROS();
});
