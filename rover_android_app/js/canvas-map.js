/**
 * Interactive High-Performance ROS 2 Canvas Map Renderer
 * Handles pan, pinch-to-zoom, occupancy grid rendering, paths, rover avatar,
 * and touch-and-drag 2D Pose / Nav Goal arrows.
 */

class CanvasMap {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    
    // Camera Transform (world to canvas)
    this.zoom = 50.0; // pixels per meter
    this.panX = 0;    // screen pixels
    this.panY = 0;    // screen pixels
    
    // Map data
    this.mapData = null;
    this.mapCanvas = null; // offscreen cached bitmap
    
    // Entities
    this.roverPose = { x: 0, y: 0, yaw: 0, valid: false };
    this.globalPath = [];
    this.scanPoints = [];
    this.waypoints = [];
    
    // Interactive Tool State
    this.activeTool = 'pan'; // 'pan', 'pose_est', 'nav_goal', 'waypoint'
    this.isDragging = false;
    this.dragStartWorld = null;
    this.dragCurrentWorld = null;
    
    // Touch gesture state (pinch-to-zoom)
    this.touchPointers = new Map();
    this.initialPinchDist = 0;
    this.initialPinchZoom = 50;
    
    // Callbacks
    this.onPoseConfirmed = null; // function(x, y, yaw, tool)
    this.onWaypointAdded = null;  // function(x, y, yaw)
    
    this.initCanvasSize();
    this.attachEvents();
    this.startRenderLoop();
  }

  initCanvasSize() {
    const resize = () => {
      const rect = this.canvas.parentElement.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      this.canvas.width = rect.width * dpr;
      this.canvas.height = rect.height * dpr;
      this.ctx.scale(dpr, dpr);
      this.displayWidth = rect.width;
      this.displayHeight = rect.height;
      if (this.panX === 0 && this.panY === 0) {
        this.panX = this.displayWidth / 2;
        this.panY = this.displayHeight / 2;
      }
    };
    window.addEventListener('resize', resize);
    resize();
  }

  // --- World to Screen & Screen to World Transforms ---
  worldToScreen(x, y) {
    // ROS: X right, Y up. Canvas: X right, Y down.
    const screenX = this.panX + x * this.zoom;
    const screenY = this.panY - y * this.zoom;
    return { x: screenX, y: screenY };
  }

  screenToWorld(screenX, screenY) {
    const x = (screenX - this.panX) / this.zoom;
    const y = (this.panY - screenY) / this.zoom;
    return { x, y };
  }

  recenter(worldX = null, worldY = null) {
    const targetX = worldX !== null ? worldX : (this.roverPose.valid ? this.roverPose.x : 0);
    const targetY = worldY !== null ? worldY : (this.roverPose.valid ? this.roverPose.y : 0);
    this.panX = this.displayWidth / 2 - targetX * this.zoom;
    this.panY = this.displayHeight / 2 + targetY * this.zoom;
  }

  setZoom(factor, centerX = this.displayWidth / 2, centerY = this.displayHeight / 2) {
    const prevWorld = this.screenToWorld(centerX, centerY);
    this.zoom = Math.max(10, Math.min(300, this.zoom * factor));
    this.panX = centerX - prevWorld.x * this.zoom;
    this.panY = centerY + prevWorld.y * this.zoom;
  }

  // --- Map Update & Offscreen Caching ---
  updateMap(gridMsg) {
    this.mapData = gridMsg;
    const info = gridMsg.info;
    const width = info.width;
    const height = info.height;
    const data = gridMsg.data;

    // Create offscreen canvas for occupancy grid
    const offCanvas = document.createElement('canvas');
    offCanvas.width = width;
    offCanvas.height = height;
    const offCtx = offCanvas.getContext('2d');
    const imgData = offCtx.createImageData(width, height);
    const pixels = imgData.data;

    for (let row = 0; row < height; row++) {
      for (let col = 0; col < width; col++) {
        // ROS grid is row-major starting at bottom-left; canvas is top-left
        const gridIdx = (height - 1 - row) * width + col;
        const pixelIdx = (row * width + col) * 4;
        const val = data[gridIdx];

        if (val === -1) {
          // Unknown space: dark blue/gray
          pixels[pixelIdx + 0] = 13;
          pixels[pixelIdx + 1] = 18;
          pixels[pixelIdx + 2] = 30;
          pixels[pixelIdx + 3] = 180;
        } else if (val === 0) {
          // Free space: clear road slate
          pixels[pixelIdx + 0] = 30;
          pixels[pixelIdx + 1] = 41;
          pixels[pixelIdx + 2] = 59;
          pixels[pixelIdx + 3] = 220;
        } else if (val >= 90) {
          // Lethal obstacle: glowing electric cyan
          pixels[pixelIdx + 0] = 0;
          pixels[pixelIdx + 1] = 229;
          pixels[pixelIdx + 2] = 255;
          pixels[pixelIdx + 3] = 255;
        } else {
          // Inflation cost: gradient from pink/purple to slate
          const ratio = val / 100.0;
          pixels[pixelIdx + 0] = Math.floor(139 * ratio + 30 * (1 - ratio));
          pixels[pixelIdx + 1] = Math.floor(92 * ratio + 41 * (1 - ratio));
          pixels[pixelIdx + 2] = Math.floor(246 * ratio + 59 * (1 - ratio));
          pixels[pixelIdx + 3] = 200;
        }
      }
    }
    offCtx.putImageData(imgData, 0, 0);
    this.mapCanvas = offCanvas;
  }

  updateGlobalPath(pathMsg) {
    if (!pathMsg || !pathMsg.poses) {
      this.globalPath = [];
      return;
    }
    this.globalPath = pathMsg.poses.map(p => ({
      x: p.pose.position.x,
      y: p.pose.position.y
    }));
  }

  updateLaserScan(scanMsg) {
    if (!scanMsg || !this.roverPose.valid) return;
    const points = [];
    const angleMin = scanMsg.angle_min;
    const angleInc = scanMsg.angle_increment;
    const ranges = scanMsg.ranges;
    const rx = this.roverPose.x;
    const ry = this.roverPose.y;
    const rYaw = this.roverPose.yaw;

    // Subsample scan points for mobile UI performance
    const step = Math.max(1, Math.floor(ranges.length / 180));
    for (let i = 0; i < ranges.length; i += step) {
      const r = ranges[i];
      if (r >= scanMsg.range_min && r <= scanMsg.range_max) {
        const theta = rYaw + angleMin + i * angleInc;
        points.push({
          x: rx + r * Math.cos(theta),
          y: ry + r * Math.sin(theta)
        });
      }
    }
    this.scanPoints = points;
  }

  // --- Rendering Loop ---
  startRenderLoop() {
    const render = () => {
      this.draw();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  draw() {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.displayWidth, this.displayHeight);

    // 1. Background Grid
    this.drawGrid(ctx);

    // 2. Occupancy Grid Map
    if (this.mapCanvas && this.mapData) {
      const info = this.mapData.info;
      const originX = info.origin.position.x;
      const originY = info.origin.position.y;
      const res = info.resolution;
      const mapW = info.width * res;
      const mapH = info.height * res;

      const topLeft = this.worldToScreen(originX, originY + mapH);
      const widthPx = mapW * this.zoom;
      const heightPx = mapH * this.zoom;

      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(this.mapCanvas, topLeft.x, topLeft.y, widthPx, heightPx);
    }

    // 3. LaserScan Points
    if (this.scanPoints.length > 0) {
      ctx.fillStyle = '#10b981'; // emerald green
      for (const pt of this.scanPoints) {
        const scr = this.worldToScreen(pt.x, pt.y);
        ctx.beginPath();
        ctx.arc(scr.x, scr.y, 2.0, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // 4. Global Nav Plan
    if (this.globalPath.length > 1) {
      ctx.beginPath();
      const first = this.worldToScreen(this.globalPath[0].x, this.globalPath[0].y);
      ctx.moveTo(first.x, first.y);
      for (let i = 1; i < this.globalPath.length; i++) {
        const pt = this.worldToScreen(this.globalPath[i].x, this.globalPath[i].y);
        ctx.lineTo(pt.x, pt.y);
      }
      ctx.strokeStyle = '#00e5ff';
      ctx.lineWidth = 3;
      ctx.shadowColor = '#00e5ff';
      ctx.shadowBlur = 8;
      ctx.stroke();
      ctx.shadowBlur = 0;
    }

    // 5. Waypoints
    this.drawWaypoints(ctx);

    // 6. Rover Footprint
    if (this.roverPose.valid) {
      this.drawRover(ctx, this.roverPose.x, this.roverPose.y, this.roverPose.yaw);
    }

    // 7. Interactive Drag Arrow (2D Pose / Nav Goal preview)
    if (this.isDragging && this.dragStartWorld && this.dragCurrentWorld) {
      this.drawInteractiveArrow(ctx, this.dragStartWorld, this.dragCurrentWorld);
    }
  }

  drawGrid(ctx) {
    const stepWorld = 1.0; // 1 meter grid
    const stepPx = stepWorld * this.zoom;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
    ctx.lineWidth = 1;

    const startX = this.panX % stepPx;
    for (let x = startX; x < this.displayWidth; x += stepPx) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, this.displayHeight);
      ctx.stroke();
    }

    const startY = this.panY % stepPx;
    for (let y = startY; y < this.displayHeight; y += stepPx) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(this.displayWidth, y);
      ctx.stroke();
    }
  }

  drawRover(ctx, wx, wy, yaw) {
    const scr = this.worldToScreen(wx, wy);
    ctx.save();
    ctx.translate(scr.x, scr.y);
    ctx.rotate(-yaw); // Canvas clockwise vs ROS counter-clockwise

    const roverRadiusPx = 0.18 * this.zoom; // 18cm rover radius
    const wheelHalfWidth = 0.14 * this.zoom; // 28cm track / 2
    const wheelLength = 0.10 * this.zoom;

    // Heading beam / FOV triangle
    ctx.fillStyle = 'rgba(0, 229, 255, 0.15)';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(roverRadiusPx * 2.2, -roverRadiusPx * 0.9);
    ctx.lineTo(roverRadiusPx * 2.2, roverRadiusPx * 0.9);
    ctx.closePath();
    ctx.fill();

    // Chassis body
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#00e5ff';
    ctx.lineWidth = 2.5;
    ctx.shadowColor = '#00e5ff';
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.arc(0, 0, roverRadiusPx, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Left and Right Wheels
    ctx.fillStyle = '#38bdf8';
    ctx.fillRect(-wheelLength / 2, -wheelHalfWidth - 4, wheelLength, 8);
    ctx.fillRect(-wheelLength / 2, wheelHalfWidth - 4, wheelLength, 8);

    // Direction arrow
    ctx.strokeStyle = '#00e5ff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-roverRadiusPx * 0.4, 0);
    ctx.lineTo(roverRadiusPx * 0.8, 0);
    ctx.lineTo(roverRadiusPx * 0.4, -roverRadiusPx * 0.4);
    ctx.moveTo(roverRadiusPx * 0.8, 0);
    ctx.lineTo(roverRadiusPx * 0.4, roverRadiusPx * 0.4);
    ctx.stroke();

    ctx.restore();
  }

  drawWaypoints(ctx) {
    for (let i = 0; i < this.waypoints.length; i++) {
      const wp = this.waypoints[i];
      const scr = this.worldToScreen(wp.x, wp.y);

      // Outer glow circle
      ctx.fillStyle = '#8b5cf6';
      ctx.beginPath();
      ctx.arc(scr.x, scr.y, 14, 0, Math.PI * 2);
      ctx.fill();

      // Heading tick
      if (wp.yaw !== undefined) {
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(scr.x, scr.y);
        ctx.lineTo(scr.x + 20 * Math.cos(-wp.yaw), scr.y + 20 * Math.sin(-wp.yaw));
        ctx.stroke();
      }

      // Waypoint index number
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`${i + 1}`, scr.x, scr.y);
    }
  }

  drawInteractiveArrow(ctx, startW, currentW) {
    const startS = this.worldToScreen(startW.x, startW.y);
    const endS = this.worldToScreen(currentW.x, currentW.y);
    const angle = Math.atan2(endS.y - startS.y, endS.x - startS.x);

    const isPoseEst = this.activeTool === 'pose_est';
    const color = isPoseEst ? '#10b981' : '#f59e0b'; // Green for pose, Orange for goal

    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 3;
    ctx.shadowColor = color;
    ctx.shadowBlur = 12;

    // Origin circle
    ctx.beginPath();
    ctx.arc(startS.x, startS.y, 8, 0, Math.PI * 2);
    ctx.fill();

    // Arrow line
    ctx.beginPath();
    ctx.moveTo(startS.x, startS.y);
    ctx.lineTo(endS.x, endS.y);
    ctx.stroke();

    // Arrow head
    const headLen = 14;
    ctx.beginPath();
    ctx.moveTo(endS.x, endS.y);
    ctx.lineTo(endS.x - headLen * Math.cos(angle - Math.PI / 6), endS.y - headLen * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(endS.x - headLen * Math.cos(angle + Math.PI / 6), endS.y - headLen * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }

  // --- Touch & Mouse Event Handling ---
  attachEvents() {
    const c = this.canvas;

    // Pointer Down
    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      this.touchPointers.set(e.pointerId, { x: e.clientX, y: e.clientY });

      if (this.touchPointers.size === 1) {
        const worldPos = this.screenToWorld(e.clientX, e.clientY);
        this.isDragging = true;
        this.dragStartWorld = worldPos;
        this.dragCurrentWorld = worldPos;
        this.dragStartScreen = { x: e.clientX, y: e.clientY };
      } else if (this.touchPointers.size === 2) {
        // Pinch zoom start
        const pts = Array.from(this.touchPointers.values());
        this.initialPinchDist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
        this.initialPinchZoom = this.zoom;
      }
    });

    // Pointer Move
    c.addEventListener('pointermove', (e) => {
      if (!this.touchPointers.has(e.pointerId)) return;
      this.touchPointers.set(e.pointerId, { x: e.clientX, y: e.clientY });

      if (this.touchPointers.size === 2) {
        // Pinch-to-zoom
        const pts = Array.from(this.touchPointers.values());
        const curDist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
        if (this.initialPinchDist > 0) {
          const ratio = curDist / this.initialPinchDist;
          const midX = (pts[0].x + pts[1].x) / 2;
          const midY = (pts[0].y + pts[1].y) / 2;
          this.setZoom(ratio * (this.initialPinchZoom / this.zoom), midX, midY);
        }
      } else if (this.isDragging) {
        if (this.activeTool === 'pan') {
          // Pan map
          const dx = e.clientX - this.dragStartScreen.x;
          const dy = e.clientY - this.dragStartScreen.y;
          this.panX += dx;
          this.panY += dy;
          this.dragStartScreen = { x: e.clientX, y: e.clientY };
        } else {
          // Arrow drag
          this.dragCurrentWorld = this.screenToWorld(e.clientX, e.clientY);
        }
      }
    });

    // Pointer Up
    const onPointerUp = (e) => {
      this.touchPointers.delete(e.pointerId);

      if (this.isDragging) {
        this.isDragging = false;
        const startW = this.dragStartWorld;
        const curW = this.dragCurrentWorld;

        if (this.activeTool === 'pose_est' || this.activeTool === 'nav_goal') {
          // Compute yaw from drag direction
          const dx = curW.x - startW.x;
          const dy = curW.y - startW.y;
          const yaw = Math.hypot(dx, dy) > 0.05 ? Math.atan2(dy, dx) : 0.0;
          if (this.onPoseConfirmed) {
            this.onPoseConfirmed(startW.x, startW.y, yaw, this.activeTool);
          }
        } else if (this.activeTool === 'waypoint') {
          if (this.onWaypointAdded) {
            this.onWaypointAdded(startW.x, startW.y, 0.0);
          }
        }
      }
      this.dragStartWorld = null;
      this.dragCurrentWorld = null;
    };

    c.addEventListener('pointerup', onPointerUp);
    c.addEventListener('pointercancel', onPointerUp);

    // Mouse Wheel Zoom for Desktop testing
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.15 : 0.85;
      this.setZoom(factor, e.clientX, e.clientY);
    }, { passive: false });
  }
}

window.CanvasMap = CanvasMap;
