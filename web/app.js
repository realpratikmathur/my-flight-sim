// 3D viewer for the quadcopter simulator. Pure display: it receives the vehicle
// state from sim/server.py over a WebSocket and draws it with three.js.
//
// Coordinates: the simulator uses ENU (x east, y north, z up); three.js is
// y-up. We map (x, y, z)_ENU -> (x, z, -y)_three, which is a rotation of
// -90 degrees about x, so quaternions map (w, x, y, z) -> (w, x, z, -y).
import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

const enuToThree = (p) => new THREE.Vector3(p[0], p[2], -p[1]);
const quatToThree = (q) => new THREE.Quaternion(q[1], q[3], -q[2], q[0]);

// ------------------------------------------------------------------ scene
const container = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x9cc3e6);
scene.fog = new THREE.Fog(0x9cc3e6, 60, 260);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 1000);
camera.position.set(-3, 1.5, 0);

scene.add(new THREE.HemisphereLight(0xdfefff, 0x4a5a3a, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(30, 60, 20);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -20, right: 20, top: 20, bottom: -20, near: 1, far: 150 });
scene.add(sun);
scene.add(sun.target);

// Ground with a grid (1 m minor lines, 10 m major lines)
const ground = new THREE.Mesh(new THREE.PlaneGeometry(1000, 1000), new THREE.MeshLambertMaterial({ color: 0x6f8f4e }));
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const grid1 = new THREE.GridHelper(200, 200, 0x5f7f40, 0x5f7f40);
grid1.position.y = 0.002;
scene.add(grid1);
const grid10 = new THREE.GridHelper(200, 20, 0x3e5a28, 0x3e5a28);
grid10.position.y = 0.003;
scene.add(grid10);

// Helipad at the origin
const pad = new THREE.Mesh(new THREE.CircleGeometry(1.2, 48), new THREE.MeshLambertMaterial({ color: 0x3a3f47 }));
pad.rotation.x = -Math.PI / 2;
pad.position.y = 0.005;
pad.receiveShadow = true;
scene.add(pad);
const ring = new THREE.Mesh(new THREE.RingGeometry(1.0, 1.1, 48), new THREE.MeshBasicMaterial({ color: 0xffd43b }));
ring.rotation.x = -Math.PI / 2;
ring.position.y = 0.007;
scene.add(ring);
const hMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
// The H reads upright from the default chase camera, which looks east (+x)
for (const [w, d, x, z] of [[0.9, 0.12, 0, -0.3], [0.9, 0.12, 0, 0.3], [0.12, 0.6, 0, 0]]) {
  const bar = new THREE.Mesh(new THREE.PlaneGeometry(w, d), hMat);
  bar.rotation.x = -Math.PI / 2;
  bar.position.set(x, 0.008, z);
  scene.add(bar);
}

// Reference poles every 10 m east and north, for a sense of distance
const poleMat = new THREE.MeshLambertMaterial({ color: 0xf08c00 });
for (let i = -5; i <= 5; i++) {
  for (const [e, n] of [[i * 10, 20], [i * 10, -20], [20, i * 10], [-20, i * 10]]) {
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 2, 8), poleMat);
    pole.position.copy(enuToThree([e, n, 1]));
    pole.castShadow = true;
    scene.add(pole);
  }
}
// North marker
const north = new THREE.Mesh(new THREE.ConeGeometry(0.4, 1.2, 16), new THREE.MeshLambertMaterial({ color: 0xc92a2a }));
north.position.copy(enuToThree([0, 25, 0.6]));
scene.add(north);

// ------------------------------------------------------------------ vehicle
// Built in body axes mapped to three.js: forward +x, left -z, up +y
const quad = new THREE.Group();
scene.add(quad);
const bodyMat = new THREE.MeshStandardMaterial({ color: 0x343a40, metalness: 0.3, roughness: 0.6 });
const body = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.05, 0.12), bodyMat);
body.castShadow = true;
quad.add(body);
const nose = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.03, 0.05), new THREE.MeshStandardMaterial({ color: 0xff6b6b }));
nose.position.set(0.1, 0.0, 0);
quad.add(nose);

const ARM = 0.17;
const A = ARM / Math.SQRT2;
// Motor order matches SimTypes.fpp: 0 front-left, 1 front-right, 2 back-right, 3 back-left
const motorPos = [[A, -A], [A, A], [-A, A], [-A, -A]];  // (three x, three z)
const motorColor = [0xff6b6b, 0xff6b6b, 0x51cf66, 0x51cf66];  // red front, green back
const props = [];
motorPos.forEach(([x, z], i) => {
  const arm = new THREE.Mesh(new THREE.BoxGeometry(ARM, 0.015, 0.02), bodyMat);
  arm.position.set(x / 2, 0, z / 2);
  arm.rotation.y = -Math.atan2(z, x);
  arm.castShadow = true;
  quad.add(arm);
  const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.035, 12),
                               new THREE.MeshStandardMaterial({ color: motorColor[i] }));
  motor.position.set(x, 0.015, z);
  motor.castShadow = true;
  quad.add(motor);
  const prop = new THREE.Group();
  prop.position.set(x, 0.037, z);
  const blade = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.004, 0.018), new THREE.MeshStandardMaterial({ color: 0xdee2e6 }));
  prop.add(blade);
  const disc = new THREE.Mesh(new THREE.CircleGeometry(0.1, 24),
                              new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0, side: THREE.DoubleSide }));
  disc.rotation.x = -Math.PI / 2;
  prop.add(disc);
  quad.add(prop);
  props.push({ group: prop, disc, angle: Math.random() * Math.PI, dir: (i % 2 === 0) ? -1 : 1 });
});
// Landing gear
for (const z of [-0.06, 0.06]) {
  const skid = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.01, 0.01), bodyMat);
  skid.position.set(0, -0.06, z);
  quad.add(skid);
}
quad.traverse((o) => { if (o.isMesh) o.castShadow = true; });

// Altitude line from the vehicle down to the ground
const dropLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
                                new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 0.1, gapSize: 0.08 }));
scene.add(dropLine);

// ------------------------------------------------------------------ cameras
let camMode = 'chase';
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enabled = false;
orbit.enableDamping = true;
const groundCamPos = new THREE.Vector3(-6, 1.6, 6);

document.querySelectorAll('[data-cam]').forEach((b) => b.addEventListener('click', () => {
  camMode = b.dataset.cam;
  document.querySelectorAll('[data-cam]').forEach((x) => x.classList.toggle('active', x === b));
  orbit.enabled = camMode === 'orbit';
  if (camMode === 'orbit') orbit.target.copy(quad.position);
}));

// ------------------------------------------------------------------ HUD
const $ = (id) => document.getElementById(id);
const motorsEl = $('motors');
const motorBars = [];
['FL', 'FR', 'BR', 'BL'].forEach((name) => {
  const m = document.createElement('div');
  m.className = 'motor';
  m.innerHTML = `<div class="bar"><div class="act"></div><div class="cmd"></div></div>${name}`;
  motorsEl.appendChild(m);
  motorBars.push({ act: m.querySelector('.act'), cmd: m.querySelector('.cmd') });
});

let ws = null;
let state = null;
const send = (obj) => { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj)); };
$('reset').addEventListener('click', () => send({ cmd: 'reset' }));
$('pause').addEventListener('click', () => send({ cmd: 'pause' }));
document.querySelectorAll('[data-speed]').forEach((b) => b.addEventListener('click', () => {
  send({ cmd: 'speed', value: parseFloat(b.dataset.speed) });
  document.querySelectorAll('[data-speed]').forEach((x) => x.classList.toggle('active', x === b));
}));

function updateHud(s) {
  const link = $('link');
  if (s.fswOnline) {
    link.className = 'status online';
    link.textContent = 'F´ flight software: ONLINE (lock-step)';
  } else {
    link.className = 'status offline';
    link.textContent = 'F´ flight software: OFFLINE (motors off)';
  }
  const horiz = Math.hypot(s.velocity[0], s.velocity[1]);
  $('simTime').textContent = `${s.simTime.toFixed(2)} s${s.paused ? ' (paused)' : ''}${s.speed !== 1 ? ` @${s.speed}×` : ''}`;
  $('alt').textContent = `${s.position[2].toFixed(2)} m`;
  $('vz').textContent = `${s.velocity[2].toFixed(2)} m/s`;
  $('speed').textContent = `${horiz.toFixed(2)} m/s`;
  $('att').textContent = `${s.tiltDeg.toFixed(1)}° / ${((90 - s.yawDeg + 360) % 360).toFixed(0)}°`;
  $('pos').textContent = `${s.position[0].toFixed(1)} / ${s.position[1].toFixed(1)} m`;
  $('batt').textContent = `${s.battery.toFixed(2)} V`;
  $('hover').textContent = s.hoverThrottle.toFixed(3);
  motorBars.forEach((bar, i) => {
    bar.act.style.height = `${(s.rotorSpeed[i] * 100).toFixed(1)}%`;
    bar.cmd.style.bottom = `${(Math.min(1, s.motorCmd[i]) * 100).toFixed(1)}%`;
  });
  const banner = $('banner');
  if (s.crashed) {
    banner.style.display = 'block';
    banner.innerHTML = `CRASHED<small>${s.crashReason}. Press "Reset vehicle" (and DISARM in the GDS).</small>`;
  } else {
    banner.style.display = 'none';
  }
}

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (ev) => { state = JSON.parse(ev.data); updateHud(state); };
  ws.onclose = () => {
    $('link').className = 'status offline';
    $('link').textContent = 'Simulator not running (start sim/server.py)';
    setTimeout(connect, 1000);
  };
}
connect();

// ------------------------------------------------------------------ render loop
const clock = new THREE.Clock();
const camTarget = new THREE.Vector3();
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(0.1, clock.getDelta());
  if (state) {
    const p = enuToThree(state.position);
    quad.position.copy(p);
    quad.quaternion.copy(quatToThree(state.attitude));
    props.forEach((pr, i) => {
      const w = state.rotorSpeed[i];
      pr.angle += pr.dir * w * 120 * dt;
      pr.group.rotation.y = pr.angle;
      pr.disc.material.opacity = Math.min(0.35, w * 0.6);
    });
    const pts = [p, new THREE.Vector3(p.x, 0.01, p.z)];
    dropLine.geometry.setFromPoints(pts);
    dropLine.computeLineDistances();
    sun.position.set(p.x + 30, 60, p.z + 20);
    sun.target.position.copy(p);

    if (camMode === 'chase') {
      const yaw = state.yawDeg * Math.PI / 180;
      const behind = new THREE.Vector3(-Math.cos(yaw) * 2.4, 0.9, Math.sin(yaw) * 2.4);
      const desired = p.clone().add(behind);
      desired.y = Math.max(desired.y, 0.3);
      camera.position.lerp(desired, 1 - Math.exp(-dt * 3));
      camTarget.lerp(p, 1 - Math.exp(-dt * 8));
      camera.lookAt(camTarget);
    } else if (camMode === 'ground') {
      camera.position.copy(groundCamPos);
      camera.lookAt(p);
    } else {
      orbit.target.lerp(p, 1 - Math.exp(-dt * 5));
      orbit.update();
    }
  }
  renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
