/*
PROTOTYPE. Path player driver: replays paths/demo.json through the same
math as path_reader_rik.py + trajectory_visualizer.py (0.005 m lerp steps
at 30 Hz, slerped orientation, 1 s dwell per waypoint), rendered as a
transport-controlled player with a two-track timeline (FEED: the color and
splat-radius the sim is fed; PATH: x/y/z over time).

Seeking moves the *emitter* along the recorded path; the fluid itself is a
stateful solve and cannot be seeked. Scrubbing while paused paints along
the path.

Drives the sim through fluid-sim.js's initFluidSim() directly (start/move/
stop on normalized [-1, 1] samples), not through the shim globals in
visual-script.js. Those globals are for the ROS-era pages only.
*/

import { initFluidSim } from './fluid-sim.js';

'use strict';

const PP = {
  PATH_FILE: '../paths/demo.json',
  STEP_SIZE: 0.005,
  SAMPLE_RATE: 30,
  DWELL_S: 1.0,
  /* timeline is read-only instrumentation (owner call 2026-08-22);
     flip to re-enable drag-the-robot-through-time scrubbing */
  SCRUB: false
};

/* ---------------- pipeline math (mirrors the ROS nodes) ---------------- */

function ppQuatSlerp(q0, q1, t) {
  let d = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];
  let q1c = q1.slice();
  if (d < 0.0) { d = -d; q1c = q1c.map(v => -v); }
  if (d > 0.9995) {
    const out = q0.map((v, i) => v + t * (q1c[i] - v));
    const n = Math.hypot(...out);
    return out.map(v => v / n);
  }
  const theta0 = Math.acos(d);
  const s0 = Math.cos(theta0 * t) - d * Math.sin(theta0 * t) / Math.sin(theta0);
  const s1 = Math.sin(theta0 * t) / Math.sin(theta0);
  return q0.map((v, i) => s0 * v + s1 * q1c[i]);
}

function ppEuler(q) {
  const [x, y, z, w] = q;
  const rx = Math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y));
  const ry = Math.asin(Math.max(-1, Math.min(1, 2 * (w*y - z*x))));
  const rz = Math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
  return [rx, ry, rz];
}

/* One sample per 1/30 s covering every segment and dwell of the path. */
function ppBuildTimeline(data) {
  const waypoints = data.path;
  const samples = [];
  const scale = a => (a + Math.PI) * 0.15 / (2 * Math.PI);
  let current = waypoints[waypoints.length - 1];  // arm starts from home

  const push = (pose, wp, dwell) => {
    const [rx, ry, rz] = ppEuler(pose.orientation);
    samples.push({
      x: pose.position[0],
      y: -pose.position[1],
      z: pose.position[2],
      color: { r: scale(rx), g: scale(ry), b: scale(rz) },
      radius: (pose.position[2] + 1) / 2,
      wp, dwell
    });
  };

  waypoints.forEach((target, wi) => {
    const dif = [0, 1, 2].map(i => target.position[i] - current.position[i]);
    const length = Math.hypot(...dif);
    if (length > 0) {
      const stepVector = dif.map(v => PP.STEP_SIZE / length * v);
      let lengthNext = 0;
      let prev = current.position;
      const positions = [];
      while (true) {
        const next = [0, 1, 2].map(i => prev[i] + stepVector[i]);
        lengthNext += PP.STEP_SIZE;
        if (lengthNext >= length) { positions.push(target.position); break; }
        positions.push(next);
        prev = next;
      }
      const steps = positions.length;
      positions.forEach((p, i) => push({
        position: p,
        orientation: steps === 1 ? target.orientation
          : ppQuatSlerp(current.orientation, target.orientation, i / (steps - 1))
      }, wi, false));
    }
    for (let i = 0; i < Math.round(PP.DWELL_S * PP.SAMPLE_RATE); i++) push(target, wi, true);
    current = target;
  });

  samples.forEach((s, i) => { s.t = i / PP.SAMPLE_RATE; });
  return samples;
}

/* ---------------- sim pointer driving (DPR-correct) ---------------- */

let sim = null;
const ppSim = {
  get ready() { return sim !== null; },
  start(s) { if (sim) sim.start(s); },
  move(s) { if (sim) sim.move(s); },
  stop() { if (sim) sim.stop(); }
};

/* ---------------- player ---------------- */

document.addEventListener('DOMContentLoaded', async () => {
  const overlay = document.getElementById('pp-overlay');
  const openBtn = document.getElementById('pp-open');
  const closeBtn = document.getElementById('pp-close');
  const playBtn = document.getElementById('pp-play');
  const restartBtn = document.getElementById('pp-restart');
  const readout = document.getElementById('pp-readout');
  const timelineEl = document.getElementById('pp-timeline');
  const tracksCanvas = document.getElementById('pp-tracks');

  const data = await (await fetch(PP.PATH_FILE)).json();
  const samples = ppBuildTimeline(data);
  const duration = samples.length / PP.SAMPLE_RATE;

  const state = { t: 0, playing: false, lastNow: null, lastIdx: -1, booted: false };

  const sampleAt = t =>
    samples[Math.max(0, Math.min(samples.length - 1, Math.floor(t * PP.SAMPLE_RATE)))];

  /* ---- track rendering ---- */
  const TRACKS = { feed: 30, gap: 5, path: 60, axis: 14 };
  const trackHeight = TRACKS.feed + TRACKS.gap + TRACKS.path + TRACKS.axis;
  let staticTracks = null;

  function renderStaticTracks(w, h, dpr) {
    const off = document.createElement('canvas');
    off.width = w; off.height = h;
    const ctx = off.getContext('2d');
    ctx.scale(dpr, dpr);
    const cw = w / dpr;

    const xAt = t => t / duration * cw;

    // FEED: orientation->RGB band (brightened; raw channels top out at
    // 0.15), splat-radius envelope from z drawn over it
    const radii = samples.map(s => s.radius);
    const rMin = Math.min(...radii), rMax = Math.max(...radii);
    for (let px = 0; px < cw; px++) {
      const s = sampleAt(px / cw * duration);
      const b = c => Math.round(c / 0.15 * 255);
      ctx.fillStyle = `rgb(${b(s.color.r)}, ${b(s.color.g)}, ${b(s.color.b)})`;
      ctx.fillRect(px, 0, 1.5, TRACKS.feed);
    }
    ctx.beginPath();
    for (let px = 0; px < cw; px++) {
      const s = sampleAt(px / cw * duration);
      const frac = rMax > rMin ? (s.radius - rMin) / (rMax - rMin) : 0.5;
      const y = TRACKS.feed - 3 - frac * (TRACKS.feed - 6);
      px === 0 ? ctx.moveTo(px, y) : ctx.lineTo(px, y);
    }
    ctx.strokeStyle = 'rgba(255,255,255,0.9)';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // PATH: x/y/z sparklines, dwell shading, waypoint ticks
    const top = TRACKS.feed + TRACKS.gap;
    ctx.fillStyle = '#fff';
    ctx.fillRect(0, top, cw, TRACKS.path);
    ctx.strokeStyle = 'rgba(17,17,17,0.25)';
    ctx.lineWidth = 1;
    ctx.strokeRect(0.5, top + 0.5, cw - 1, TRACKS.path - 1);

    // dwell regions
    ctx.fillStyle = 'rgba(17,17,17,0.07)';
    let dwellStart = null;
    samples.forEach((s, i) => {
      if (s.dwell && dwellStart === null) dwellStart = s.t;
      if ((!s.dwell || i === samples.length - 1) && dwellStart !== null) {
        ctx.fillRect(xAt(dwellStart), top, xAt(s.t) - xAt(dwellStart), TRACKS.path);
        dwellStart = null;
      }
    });

    // waypoint ticks at segment boundaries
    ctx.strokeStyle = 'rgba(17,17,17,0.35)';
    ctx.fillStyle = 'rgba(17,17,17,0.55)';
    ctx.font = '8px sans-serif';
    ctx.textAlign = 'center';
    samples.forEach((s, i) => {
      if (i > 0 && s.wp !== samples[i - 1].wp) {
        ctx.beginPath();
        ctx.moveTo(xAt(s.t), top);
        ctx.lineTo(xAt(s.t), top + TRACKS.path);
        ctx.stroke();
        ctx.fillText(String(s.wp + 1), xAt(s.t), top + 8);
      }
    });

    // series, each normalized to its own range for legibility
    const series = [
      { key: 'x', color: '#d98a1f' },
      { key: 'y', color: '#4a7c8c' },
      { key: 'z', color: '#7a5aa0' }
    ];
    series.forEach(({ key, color }) => {
      const vals = samples.map(s => s[key]);
      const vMin = Math.min(...vals), vMax = Math.max(...vals);
      ctx.beginPath();
      for (let px = 0; px < cw; px++) {
        const s = sampleAt(px / cw * duration);
        const frac = vMax > vMin ? (s[key] - vMin) / (vMax - vMin) : 0.5;
        const y = top + TRACKS.path - 5 - frac * (TRACKS.path - 10);
        px === 0 ? ctx.moveTo(px, y) : ctx.lineTo(px, y);
      }
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    });
    // series key, right edge
    ctx.font = 'bold 9px sans-serif';
    ctx.textAlign = 'right';
    series.forEach(({ key, color }, i) => {
      ctx.fillStyle = color;
      ctx.fillText(key.toUpperCase(), cw - 4 - i * 14, top + TRACKS.path - 4);
    });

    // time axis
    const axisTop = top + TRACKS.path;
    ctx.fillStyle = 'rgba(17,17,17,0.5)';
    ctx.strokeStyle = 'rgba(17,17,17,0.3)';
    ctx.font = '8px sans-serif';
    ctx.textAlign = 'left';
    for (let t = 0; t < duration; t += 2) {
      ctx.beginPath();
      ctx.moveTo(xAt(t), axisTop + 2);
      ctx.lineTo(xAt(t), axisTop + 6);
      ctx.stroke();
      ctx.fillText(`${t}s`, xAt(t) + 2, axisTop + 11);
    }
    return off;
  }

  function layoutTracks() {
    const dpr = window.devicePixelRatio || 1;
    const w = timelineEl.clientWidth;
    tracksCanvas.style.height = `${trackHeight}px`;
    tracksCanvas.width = Math.floor(w * dpr);
    tracksCanvas.height = Math.floor(trackHeight * dpr);
    staticTracks = renderStaticTracks(tracksCanvas.width, tracksCanvas.height, dpr);
    document.getElementById('pp-feed-label').style.top = '2px';
    document.getElementById('pp-path-label').style.top = `${TRACKS.feed + TRACKS.gap + 2}px`;
  }

  function drawTracks() {
    if (!staticTracks) return;
    const ctx = tracksCanvas.getContext('2d');
    ctx.clearRect(0, 0, tracksCanvas.width, tracksCanvas.height);
    ctx.drawImage(staticTracks, 0, 0);
    const dpr = window.devicePixelRatio || 1;
    const px = state.t / duration * (tracksCanvas.width / dpr);
    ctx.save();
    ctx.scale(dpr, dpr);
    ctx.strokeStyle = '#111';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px, 0);
    ctx.lineTo(px, trackHeight - TRACKS.axis);
    ctx.stroke();
    ctx.fillStyle = '#111';
    ctx.beginPath();
    ctx.moveTo(px - 4, 0);
    ctx.lineTo(px + 4, 0);
    ctx.lineTo(px, 5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function updateReadout() {
    readout.textContent =
      `T+${state.t.toFixed(1).padStart(4, '0')} / ${duration.toFixed(1)}s`;
  }

  /* ---- playback ---- */
  function applyCurrent(useStart) {
    const idx = Math.max(0, Math.min(samples.length - 1, Math.floor(state.t * PP.SAMPLE_RATE)));
    if (idx === state.lastIdx && !useStart) return;
    state.lastIdx = idx;
    useStart ? ppSim.start(samples[idx]) : ppSim.move(samples[idx]);
  }

  function tick(now) {
    if (state.playing) {
      if (state.lastNow != null) {
        state.t += Math.min(0.1, (now - state.lastNow) / 1000);
        if (state.t >= duration) {
          state.t -= duration;
          applyCurrent(true);   // teleport, don't smear the wrap
        } else {
          applyCurrent(false);
        }
      }
      state.lastNow = now;
    }
    drawTracks();
    updateReadout();
    requestAnimationFrame(tick);
  }

  function setPlaying(playing) {
    state.playing = playing;
    state.lastNow = null;
    playBtn.innerHTML = playing ? '&#10074;&#10074;' : '&#9654;&#xFE0E;';
    playBtn.setAttribute('aria-pressed', String(playing));
    if (playing) applyCurrent(true);
    else ppSim.stop();
  }

  /* ---- sim boot: init the module once the viewbox has layout ---- */
  function boot() {
    if (state.booted) return;
    state.booted = true;
    sim = initFluidSim(document.querySelector('.pp-viewbox canvas'));
    setPlaying(true);
  }

  /* ---- wiring ---- */
  openBtn.addEventListener('click', () => {
    overlay.removeAttribute('data-hidden');
    layoutTracks();
    if (state.booted) setPlaying(true);
    else boot();
  });
  if (new URLSearchParams(location.search).has('autorun')) openBtn.click();
  const close = () => {
    setPlaying(false);
    overlay.setAttribute('data-hidden', '');
  };
  closeBtn.addEventListener('click', close);
  overlay.addEventListener('click', e => { if (e.target === overlay) close(); });
  document.addEventListener('keydown', e => {
    if (overlay.hasAttribute('data-hidden')) return;
    if (e.key === 'Escape') close();
    if (e.key === ' ') { e.preventDefault(); setPlaying(!state.playing); }
  });

  playBtn.addEventListener('click', () => setPlaying(!state.playing));
  restartBtn.addEventListener('click', () => {
    state.t = 0;
    applyCurrent(true);
  });

  /* scrub (off by default, PP.SCRUB): pause the clock while dragging; the
     emitter paints along the path as it is dragged (the fluid cannot be
     seeked, only the input) */
  if (PP.SCRUB) {
    let scrub = { active: false, wasPlaying: false };
    const seekFromEvent = e => {
      const rect = timelineEl.getBoundingClientRect();
      const frac = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
      state.t = frac * duration;
    };
    timelineEl.addEventListener('pointerdown', e => {
      scrub.active = true;
      scrub.wasPlaying = state.playing;
      state.playing = false;
      timelineEl.setPointerCapture(e.pointerId);
      seekFromEvent(e);
      applyCurrent(true);
    });
    timelineEl.addEventListener('pointermove', e => {
      if (!scrub.active) return;
      seekFromEvent(e);
      applyCurrent(false);
    });
    timelineEl.addEventListener('pointerup', () => {
      scrub.active = false;
      setPlaying(scrub.wasPlaying);
    });
  } else {
    timelineEl.style.cursor = 'default';
  }

  window.addEventListener('resize', () => {
    if (!overlay.hasAttribute('data-hidden')) layoutTracks();
  });

  updateReadout();
  requestAnimationFrame(tick);
});
