/**
 * Virtual Analog Touch Joystick for Mobile Teleop
 * Maps touch displacements to Twist (linear.x, angular.z) with deadband,
 * rate-limited publishing, and safe auto-centering.
 */

class VirtualJoystick {
  constructor(baseId, stickId, onTwistCallback) {
    this.base = document.getElementById(baseId);
    this.stick = document.getElementById(stickId);
    this.onTwist = onTwistCallback;

    this.maxRadius = 45; // max px displacement
    this.deadband = 0.08; // 8% center deadband
    this.maxLinearSpeed = 0.12; // m/s
    this.maxAngularSpeed = 0.58; // rad/s

    this.touchId = null;
    this.baseCenter = { x: 0, y: 0 };
    this.currentTwist = { linear: 0, angular: 0 };
    
    this.publishInterval = null;
    this.init();
  }

  init() {
    const b = this.base;

    b.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      b.setPointerCapture(e.pointerId);
      this.touchId = e.pointerId;

      const rect = b.getBoundingClientRect();
      this.baseCenter = {
        x: rect.left + rect.width / 2,
        y: rect.top + rect.height / 2
      };

      this.handlePointer(e.clientX, e.clientY);
      this.startPublishing();
    });

    b.addEventListener('pointermove', (e) => {
      if (e.pointerId !== this.touchId) return;
      e.preventDefault();
      this.handlePointer(e.clientX, e.clientY);
    });

    const onPointerEnd = (e) => {
      if (e.pointerId !== this.touchId) return;
      this.touchId = null;
      this.reset();
    };

    b.addEventListener('pointerup', onPointerEnd);
    b.addEventListener('pointercancel', onPointerEnd);
  }

  handlePointer(clientX, clientY) {
    const rawDx = clientX - this.baseCenter.x;
    const rawDy = clientY - this.baseCenter.y;
    const dist = Math.hypot(rawDx, rawDy);

    const clampedDist = Math.min(dist, this.maxRadius);
    const angle = Math.atan2(rawDy, rawDx);

    const stickX = clampedDist * Math.cos(angle);
    const stickY = clampedDist * Math.sin(angle);

    // Visual stick move
    this.stick.style.transform = `translate(${stickX}px, ${stickY}px)`;

    // Normalized coordinates (-1.0 to 1.0)
    let normX = stickX / this.maxRadius;
    let normY = stickY / this.maxRadius;

    // Deadband check
    const normDist = clampedDist / this.maxRadius;
    if (normDist < this.deadband) {
      normX = 0;
      normY = 0;
    }

    // ROS Convention:
    // Drag Up (negative Y screen) -> Forward (+linear.x)
    // Drag Left (negative X screen) -> Counter-Clockwise (+angular.z)
    this.currentTwist.linear = -normY * this.maxLinearSpeed;
    this.currentTwist.angular = -normX * this.maxAngularSpeed;
  }

  startPublishing() {
    if (this.publishInterval) return;
    this.publishInterval = setInterval(() => {
      if (this.onTwist) {
        this.onTwist(this.currentTwist.linear, this.currentTwist.angular);
      }
    }, 66); // ~15 Hz
  }

  reset() {
    this.stick.style.transform = 'translate(0px, 0px)';
    this.currentTwist = { linear: 0, angular: 0 };
    if (this.onTwist) {
      this.onTwist(0, 0);
    }
    if (this.publishInterval) {
      clearInterval(this.publishInterval);
      this.publishInterval = null;
    }
  }

  setMaxLinearSpeed(val) {
    this.maxLinearSpeed = Math.max(0.02, Math.min(0.25, val));
  }
}

window.VirtualJoystick = VirtualJoystick;
