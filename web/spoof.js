/*
Spoof driver — stands in for the ROS side of the demo.

Replays paths/demo.json through the same math the real pipeline uses:
path_reader_rik.py's linear interpolation (0.005 m steps at 30 Hz, slerped
orientation, 1 s dwell per waypoint) feeding trajectory_visualizer.py's
pose-to-visualization mapping (x,y -> screen offsets, z -> splat radius,
euler angles -> RGB). Drives the same visualize_* API that ros-node.js does,
so the fluid behaves exactly as it would with the robot in the loop.
*/

'use strict';

const SPOOF = {
  PATH_FILE: '../paths/demo.json',
  STEP_SIZE: 0.005,        // m per interpolation step (DEFAULT_STEP_SIZE)
  PUBLISH_RATE: 30,        // Hz (DEFAULT_PUBLISH_RATE)
  POSE_SWITCH_DELAY: 1.0,  // s dwell at each waypoint (DEFAULT_POSE_SWITCH_DELAY)
  REPEAT: true
};

function quatSlerp(q0, q1, t) {
  let d = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];
  let q1c = q1.slice();
  if (d < 0.0) {
    d = -d;
    q1c = q1c.map(v => -v);
  }
  if (d > 0.9995) {
    let out = q0.map((v, i) => v + t * (q1c[i] - v));
    let n = Math.hypot(...out);
    return out.map(v => v / n);
  }
  let theta0 = Math.acos(d);
  let theta = theta0 * t;
  let s0 = Math.cos(theta) - d * Math.sin(theta) / Math.sin(theta0);
  let s1 = Math.sin(theta) / Math.sin(theta0);
  return q0.map((v, i) => s0 * v + s1 * q1c[i]);
}

// tf.transformations.euler_from_quaternion default 'sxyz' convention,
// quaternion stored [x, y, z, w] as in the path files
function eulerFromQuaternion(q) {
  const [x, y, z, w] = q;
  const rx = Math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y));
  let sinp = 2 * (w*y - z*x);
  sinp = Math.max(-1, Math.min(1, sinp));
  const ry = Math.asin(sinp);
  const rz = Math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
  return [rx, ry, rz];
}

// trajectory_visualizer.py pose -> config + offsets
function poseToVisualization(pose) {
  const [rx, ry, rz] = eulerFromQuaternion(pose.orientation);
  const scale = a => (a + Math.PI) * 0.15 / (2 * Math.PI);
  return {
    config: {
      COLOR: { r: scale(rx), g: scale(ry), b: scale(rz) },
      SPLAT_RADIUS: (pose.position[2] + 1) / 2
    },
    offsetX: pose.position[0],
    offsetY: -pose.position[1]
  };
}

// path_reader_rik.py _generate_linear_path, minus the hold poses (the real
// visualizer suppresses updates when the pose isn't changing; the dwell
// below covers that)
function generateLinearPath(current, target) {
  const dif = [0, 1, 2].map(i => target.position[i] - current.position[i]);
  const length = Math.hypot(...dif);
  if (length <= 0) return [target];

  const stepVector = dif.map(v => SPOOF.STEP_SIZE / length * v);
  const positions = [];
  let lengthNext = 0;
  let prev = current.position;
  while (true) {
    const next = [0, 1, 2].map(i => prev[i] + stepVector[i]);
    lengthNext += SPOOF.STEP_SIZE;
    if (lengthNext >= length) {
      positions.push(target.position);
      break;
    }
    positions.push(next);
    prev = next;
  }

  const steps = positions.length;
  return positions.map((p, i) => ({
    position: p,
    orientation: steps === 1
      ? target.orientation
      : quatSlerp(current.orientation, target.orientation, i / (steps - 1))
  }));
}

document.addEventListener('DOMContentLoaded', async function () {
  const data = await (await fetch(SPOOF.PATH_FILE)).json();
  const waypoints = data.path;

  // same posture as the demo: the arm starts from home, which the path
  // returns to as its final waypoint
  let currentPose = waypoints[waypoints.length - 1];

  const first = poseToVisualization(currentPose);
  visualize_setConfig(first.config);
  visualize_trajectoryStart(first.offsetX, first.offsetY);

  const tickMs = 1000 / SPOOF.PUBLISH_RATE;

  window.SPOOF_LOOP = 0;

  do {
    window.SPOOF_LOOP += 1;
    for (const target of waypoints) {
      const segment = generateLinearPath(currentPose, target);
      for (const pose of segment) {
        const v = poseToVisualization(pose);
        visualize_setConfig(v.config);
        visualize_trajectoryUpdate(v.offsetX, v.offsetY);
        await new Promise(r => setTimeout(r, tickMs));
      }
      currentPose = target;
      await new Promise(r => setTimeout(r, SPOOF.POSE_SWITCH_DELAY * 1000));
    }
  } while (SPOOF.REPEAT);

  visualize_trajectoryStop();
});
