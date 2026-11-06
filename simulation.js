/**
 * Multi-Sensor Obstacle Detection & Collision Prevention System
 * WebGL 3D Simulation & Hardware Digital Logic Engine
 * 
 * Authors: Cabanit, Rebecca C. | Nombrado, Gillian Threse G. | Sto Domingo, Eulyn Vincee Ann A.
 * Academic Year: 2026–2027
 */

/* ==========================================================================
   1. CONFIGURATION & HARDWARE CONSTANTS
   ========================================================================== */

/** Ultrasonic HC-SR04 detection distance threshold (scene units) */
const FRONT_THRESHOLD = 25.0;

/** Left & Right IR Proximity Sensor detection distance threshold (scene units) */
const SIDE_THRESHOLD = 15.0;

/**
 * 74HC Digital Hardware Logic Matrix & Truth Table:
 * Maps 3-bit sensor state [Front, Left, Right] to:
 * - 74HC148 / DFF Priority State (q)
 * - 74HC283 Adder obstacle count
 * - 74HC153 MUX PWM speed duty cycle
 * - 74HC139 DEMUX steering/drive direction
 */
const TRUTH_TABLE = [
  { F: 0, L: 0, R: 0, q: '00', count: '00', pwm: 1.00, pwmText: '100%', demux: 'Forward',    action: 'Full Speed Straight Forward',      stateName: 'STATE 1 (000)' },
  { F: 0, L: 0, R: 1, q: '01', count: '01', pwm: 0.75, pwmText: '75%',  demux: 'Turn Left',  action: 'Moderate Speed Pivot Turn Left',   stateName: 'STATE 2 (001)' },
  { F: 0, L: 1, R: 0, q: '10', count: '01', pwm: 0.75, pwmText: '75%',  demux: 'Turn Right', action: 'Moderate Speed Pivot Turn Right',  stateName: 'STATE 3 (010)' },
  { F: 0, L: 1, R: 1, q: '00', count: '10', pwm: 0.50, pwmText: '50%',  demux: 'Forward',    action: 'Half Speed Narrow Passage FWD',    stateName: 'STATE 4 (011)' },
  { F: 1, L: 0, R: 0, q: '10', count: '01', pwm: 0.75, pwmText: '75%',  demux: 'Turn Right', action: 'Pivot Right Away from Front',     stateName: 'STATE 5 (100)' },
  { F: 1, L: 0, R: 1, q: '01', count: '10', pwm: 0.50, pwmText: '50%',  demux: 'Turn Left',  action: 'Half Speed Turn Left (Front/R)',   stateName: 'STATE 6 (101)' },
  { F: 1, L: 1, R: 0, q: '10', count: '10', pwm: 0.50, pwmText: '50%',  demux: 'Turn Right', action: 'Half Speed Turn Right (Front/L)',  stateName: 'STATE 7 (110)' },
  { F: 1, L: 1, R: 1, q: '11', count: '11', pwm: 0.00, pwmText: '0%',   demux: 'Stop',       action: 'Emergency Hardware Brake Halt',     stateName: 'STATE 8 (111)' }
];

/* ==========================================================================
   2. GLOBAL STATE & KINEMATICS
   ========================================================================== */

/** Three.js Core Rendering Objects */
let scene, camera, renderer, controls;

/** Vehicle 3D Model Hierarchies & Visualizers */
let carGroup, leftWheel, rightWheel, casterWheel;
let frontConeMesh, leftLineMesh, rightLineMesh;
let frontUpRayMesh, frontDownRayMesh;

/** Simulation World Objects & Raycaster */
let obstacles = [];
let raycaster = new THREE.Raycaster();

/** Simulation Configuration & Modes */
let isCameraFollow = false;
let controlMode = 'AUTO';       // 'AUTO' (Autonomous IC Logic) or 'MANUAL' (WASD)
let forcedStateIndex = null;    // null for auto raycast, 0-7 for manual truth override
let simSpeedMultiplier = 1.0;

/** Vehicle Kinematic State */
let vehicleState = {
  x: 0,
  y: 0,
  z: 0,
  heading: 0,
  pitch: 0,
  roll: 0,
  linearVelocity: 0,
  angularVelocity: 0,
  targetSpeed: 0,
  targetTurn: 0
};

/** Keyboard Input Tracking for Manual WASD Mode */
let keysPressed = {};
window.addEventListener('keydown', (e) => { keysPressed[e.key.toLowerCase()] = true; });
window.addEventListener('keyup', (e) => { keysPressed[e.key.toLowerCase()] = false; });

/* ==========================================================================
   3. DOM HELPER UTILITIES
   ========================================================================== */

/**
 * Safely update innerHTML for an element by ID
 * @param {string} id - HTML element ID
 * @param {string} val - Value or markup to assign
 */
function setElText(id, val) {
  const el = document.getElementById(id);
  if (el) el.innerHTML = val;
}

/* ==========================================================================
   4. ENGINE INITIALIZATION & 3D ENVIRONMENT
   ========================================================================== */

/**
 * Initializes the Three.js scene, camera, WebGL renderer, lights, arena,
 * vehicle model, default obstacle preset, and begins the animation loop.
 */
function initEngine() {
  const container = document.getElementById('canvas-container');
  
  // Scene & Atmosphere
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x090d16);
  scene.fog = new THREE.FogExp2(0x090d16, 0.007);

  // Camera & Orbit Controls
  camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 1000);
  camera.position.set(0, 38, 58);

  // WebGL Renderer Setup
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2 - 0.02;

  // Lighting Rig
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 1.3);
  dirLight.position.set(45, 65, 25);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.camera.near = 10;
  dirLight.shadow.camera.far = 160;
  const d = 60;
  dirLight.shadow.camera.left = -d;
  dirLight.shadow.camera.right = d;
  dirLight.shadow.camera.top = d;
  dirLight.shadow.camera.bottom = -d;
  scene.add(dirLight);

  const cyanPointLight = new THREE.PointLight(0x06b6d4, 1.5, 80);
  cyanPointLight.position.set(0, 30, 0);
  scene.add(cyanPointLight);

  // Construct World Arena & Vehicle
  createEnvironmentArena();
  createVehicleModel();

  // Load Initial Course Preset & Enable Interactivity
  loadArenaPreset('OVERHANG_COURSE');
  setupObstacleInteractivity();

  // Event Listeners & Start Loop
  window.addEventListener('resize', onWindowResize);
  requestAnimationFrame(animateLoop);
}

/**
 * Builds the planar floor, grid helper, and 4 boundary walls of the test arena.
 */
function createEnvironmentArena() {
  const terrainSize = 130;

  // Floor Plane
  const terrainGeometry = new THREE.PlaneGeometry(terrainSize, terrainSize);
  terrainGeometry.rotateX(-Math.PI / 2);

  const terrainMat = new THREE.MeshStandardMaterial({
    color: 0x0f172a,
    roughness: 0.85,
    metalness: 0.15
  });

  const floorMesh = new THREE.Mesh(terrainGeometry, terrainMat);
  floorMesh.receiveShadow = true;
  scene.add(floorMesh);

  // Cyan Visual Coordinate Grid
  const gridHelper = new THREE.GridHelper(130, 65, 0x06b6d4, 0x1e293b);
  gridHelper.position.y = 0.02;
  scene.add(gridHelper);

  // Arena Perimeter Enclosure Walls
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0x1e293b,
    metalness: 0.4,
    roughness: 0.6,
    transparent: true,
    opacity: 0.75
  });
  const wallThickness = 2;
  const wallHeight = 10;
  const size = 130;

  const walls = [
    { w: size, h: wallHeight, d: wallThickness, x: 0, z: -size / 2 },
    { w: size, h: wallHeight, d: wallThickness, x: 0, z: size / 2 },
    { w: wallThickness, h: wallHeight, d: size, x: -size / 2, z: 0 },
    { w: wallThickness, h: wallHeight, d: size, x: size / 2, z: 0 }
  ];

  walls.forEach(b => {
    const wall = new THREE.Mesh(new THREE.BoxGeometry(b.w, b.h, b.d), wallMat);
    wall.position.set(b.x, b.h / 2, b.z);
    wall.receiveShadow = true;
    wall.castShadow = true;
    scene.add(wall);
  });
}

/**
 * Floor height sampler for planar movement
 * @returns {number} Constant 0 elevation
 */
function getTerrainHeight(x, z) {
  return 0;
}

/**
 * Constructs the 3D smart robotic vehicle:
 * - Dual-deck acrylic chassis with standoffs
 * - Left/right drive wheels and front caster
 * - Hardware components: ESP32, L298N driver, LM2596 buck converter, Breadboard
 * - Sensors: Ultrasonic HC-SR04, Left/Right IR proximity sensors
 * - Sensor raycast visualizers (cone mesh, pitch cylinders, IR lines)
 */
function createVehicleModel() {
  carGroup = new THREE.Group();

  // Dual Acrylic Decks
  const deckMat = new THREE.MeshPhysicalMaterial({
    color: 0x0f172a,
    transmission: 0.65,
    opacity: 0.9,
    transparent: true,
    roughness: 0.15,
    ior: 1.5
  });

  const lowerDeck = new THREE.Mesh(new THREE.BoxGeometry(10, 0.4, 18), deckMat);
  lowerDeck.position.y = 2.0;
  lowerDeck.castShadow = true;
  carGroup.add(lowerDeck);

  const upperDeck = new THREE.Mesh(new THREE.BoxGeometry(10, 0.4, 16), deckMat);
  upperDeck.position.set(0, 4.5, -1);
  upperDeck.castShadow = true;
  carGroup.add(upperDeck);

  // Brass Standoff Pillars
  const pillarMat = new THREE.MeshStandardMaterial({ color: 0xd97706, metalness: 0.85, roughness: 0.2 });
  const pillarGeo = new THREE.CylinderGeometry(0.15, 0.15, 2.1);
  [[-4.2, 3.15, -7], [4.2, 3.15, -7], [-4.2, 3.15, 6], [4.2, 3.15, 6]].forEach(pos => {
    const pillar = new THREE.Mesh(pillarGeo, pillarMat);
    pillar.position.set(...pos);
    carGroup.add(pillar);
  });

  // Drive Wheels (Left & Right Differential)
  const wheelGeo = new THREE.CylinderGeometry(2.0, 2.0, 1.2, 24);
  const tireMat = new THREE.MeshStandardMaterial({ color: 0x18181b, roughness: 0.9 });
  const rimMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.5 });

  leftWheel = new THREE.Group();
  const lwTire = new THREE.Mesh(wheelGeo, tireMat);
  lwTire.rotation.z = Math.PI / 2;
  lwTire.castShadow = true;
  const lwRim = new THREE.Mesh(new THREE.CylinderGeometry(1.2, 1.2, 1.25, 12), rimMat);
  lwRim.rotation.z = Math.PI / 2;
  leftWheel.add(lwTire, lwRim);
  leftWheel.position.set(-5.6, 2.0, 3.0);
  carGroup.add(leftWheel);

  rightWheel = new THREE.Group();
  const rwTire = new THREE.Mesh(wheelGeo, tireMat);
  rwTire.rotation.z = Math.PI / 2;
  rwTire.castShadow = true;
  const rwRim = new THREE.Mesh(new THREE.CylinderGeometry(1.2, 1.2, 1.25, 12), rimMat);
  rwRim.rotation.z = Math.PI / 2;
  rightWheel.add(rwTire, rwRim);
  rightWheel.position.set(5.6, 2.0, 3.0);
  carGroup.add(rightWheel);

  // Front Omnidirectional Caster Wheel
  const casterMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.9, roughness: 0.1 });
  casterWheel = new THREE.Mesh(new THREE.SphereGeometry(1.0, 16, 16), casterMat);
  casterWheel.position.set(0, 1.0, -7.0);
  casterWheel.castShadow = true;
  carGroup.add(casterWheel);

  // Electronics Hardware Boards
  const esp32Board = new THREE.Mesh(new THREE.BoxGeometry(3.0, 0.3, 5.0), new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.3 }));
  esp32Board.position.set(-2.6, 4.8, -2.5);
  carGroup.add(esp32Board);

  const l298Board = new THREE.Mesh(new THREE.BoxGeometry(4.0, 0.3, 4.0), new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.4 }));
  l298Board.position.set(2.6, 4.8, -2.5);
  carGroup.add(l298Board);

  const lm2596Board = new THREE.Mesh(new THREE.BoxGeometry(2.5, 0.3, 4.0), new THREE.MeshStandardMaterial({ color: 0x1e3a8a, roughness: 0.3 }));
  lm2596Board.position.set(-2.8, 4.8, 1.8);
  carGroup.add(lm2596Board);

  const bbBoard = new THREE.Mesh(new THREE.BoxGeometry(8.5, 0.3, 3.5), new THREE.MeshStandardMaterial({ color: 0xf8fafc }));
  bbBoard.position.set(0, 4.8, 4.5);
  carGroup.add(bbBoard);

  // Sensor Mounts
  const hcBoard = new THREE.Mesh(new THREE.BoxGeometry(4.5, 2.0, 0.3), new THREE.MeshStandardMaterial({ color: 0x0284c7 }));
  hcBoard.position.set(0, 3.2, 9.0);
  carGroup.add(hcBoard);

  const irMat = new THREE.MeshStandardMaterial({ color: 0x0f172a });
  const leftIR = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.4, 2.5), irMat);
  leftIR.position.set(-4.5, 2.8, 7.5);
  leftIR.rotation.y = -Math.PI / 6;
  carGroup.add(leftIR);

  const rightIR = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.4, 2.5), irMat);
  rightIR.position.set(4.5, 2.8, 7.5);
  rightIR.rotation.y = Math.PI / 6;
  carGroup.add(rightIR);

  // Ultrasonic FOV Cone & Multi-Pitch Beams
  const coneGeo = new THREE.ConeGeometry(Math.tan(Math.PI / 6) * FRONT_THRESHOLD, FRONT_THRESHOLD, 16, 1, true);
  coneGeo.rotateX(-Math.PI / 2);
  coneGeo.translate(0, 0, FRONT_THRESHOLD / 2);
  frontConeMesh = new THREE.Mesh(coneGeo, new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.2, wireframe: true }));
  frontConeMesh.position.set(0, 3.2, 9.0);
  carGroup.add(frontConeMesh);

  frontUpRayMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, FRONT_THRESHOLD), new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.6 }));
  frontUpRayMesh.rotation.x = Math.PI / 2 - 0.25;
  frontUpRayMesh.position.set(0, 3.8, 9.0 + FRONT_THRESHOLD / 2);
  carGroup.add(frontUpRayMesh);

  frontDownRayMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, FRONT_THRESHOLD), new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.6 }));
  frontDownRayMesh.rotation.x = Math.PI / 2 + 0.15;
  frontDownRayMesh.position.set(0, 2.6, 9.0 + FRONT_THRESHOLD / 2);
  carGroup.add(frontDownRayMesh);

  // IR Raycasting Indicator Lines (Angled 30 Degrees Left & Right)
  leftLineMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, SIDE_THRESHOLD), new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.7 }));
  leftLineMesh.rotation.x = Math.PI / 2;
  leftLineMesh.rotation.z = -Math.PI / 6;
  leftLineMesh.position.set(-4.5, 2.8, 7.5 + SIDE_THRESHOLD / 2);
  carGroup.add(leftLineMesh);

  rightLineMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, SIDE_THRESHOLD), new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.7 }));
  rightLineMesh.rotation.x = Math.PI / 2;
  rightLineMesh.rotation.z = Math.PI / 6;
  rightLineMesh.position.set(4.5, 2.8, 7.5 + SIDE_THRESHOLD / 2);
  carGroup.add(rightLineMesh);

  carGroup.position.set(0, 0, 0);
  scene.add(carGroup);
}

/* ==========================================================================
   5. OBSTACLE MANAGEMENT & USER INTERACTIVITY
   ========================================================================== */

/**
 * Enables mouse drag-and-drop on obstacles across the ground plane.
 */
function setupObstacleInteractivity() {
  const raycasterDrag = new THREE.Raycaster();
  const mouse = new THREE.Vector2();
  let selectedObject = null;
  let plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  let planeIntersect = new THREE.Vector3();

  const dom = renderer.domElement;

  dom.addEventListener('mousedown', (e) => {
    if (e.button !== 0) return;
    const rect = dom.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    raycasterDrag.setFromCamera(mouse, camera);
    const intersects = raycasterDrag.intersectObjects(obstacles, true);

    if (intersects.length > 0) {
      let rootObj = intersects[0].object;
      while (rootObj.parent && rootObj.parent !== scene) {
        rootObj = rootObj.parent;
      }
      selectedObject = rootObj;
      controls.enabled = false;
    }
  });

  dom.addEventListener('mousemove', (e) => {
    if (!selectedObject) return;
    const rect = dom.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    raycasterDrag.setFromCamera(mouse, camera);
    if (raycasterDrag.ray.intersectPlane(plane, planeIntersect)) {
      selectedObject.position.x = planeIntersect.x;
      selectedObject.position.z = planeIntersect.z;
    }
  });

  window.addEventListener('mouseup', () => {
    selectedObject = null;
    controls.enabled = true;
  });
}

/**
 * Spawns a standard solid rectangular block obstacle.
 */
function spawnObstacle(x = (Math.random() - 0.5) * 44, z = (Math.random() - 0.5) * 44 + 10, w = 6, h = 8, d = 6) {
  const box = new THREE.Mesh(
    new THREE.BoxGeometry(w, h, d),
    new THREE.MeshStandardMaterial({ color: 0x64748b, roughness: 0.4, metalness: 0.3 })
  );
  box.position.set(x, h / 2, z);
  box.castShadow = true;
  box.receiveShadow = true;
  scene.add(box);
  obstacles.push(box);
}

/**
 * Spawns an elevated overhang bridge structure for testing vertical sensor clearance.
 */
function spawnOverhangBridge(x = 0, z = 20, isLow = true) {
  const bridgeGroup = new THREE.Group();
  const clearanceY = isLow ? 3.5 : 8.0;

  const deck = new THREE.Mesh(
    new THREE.BoxGeometry(22, 2.0, 8),
    new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.5, roughness: 0.3 })
  );
  deck.position.set(0, clearanceY + 1.0, 0);
  deck.castShadow = true;
  bridgeGroup.add(deck);

  const pillarMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.7 });
  const pillarL = new THREE.Mesh(new THREE.BoxGeometry(2, clearanceY + 2, 8), pillarMat);
  pillarL.position.set(-10, (clearanceY + 2) / 2, 0);
  const pillarR = pillarL.clone();
  pillarR.position.set(10, (clearanceY + 2) / 2, 0);
  bridgeGroup.add(pillarL, pillarR);

  bridgeGroup.position.set(x, 0, z);
  scene.add(bridgeGroup);
  obstacles.push(bridgeGroup);
}

/**
 * Spawns a low ground bump barrier.
 */
function spawnGroundBump(x = 0, z = 10) {
  const bumpHeight = 2.8;
  const bump = new THREE.Mesh(
    new THREE.BoxGeometry(18, bumpHeight, 3.5),
    new THREE.MeshStandardMaterial({ color: 0xb45309, roughness: 0.6 })
  );
  bump.position.set(x, bumpHeight / 2, z);
  bump.castShadow = true;
  scene.add(bump);
  obstacles.push(bump);
}

/**
 * Spawns an inclined ramp structure.
 */
function spawnRampStructure(x = 0, z = 15) {
  const rampGroup = new THREE.Group();
  const rampGeo = new THREE.BoxGeometry(14, 0.8, 24);
  const rampMesh = new THREE.Mesh(rampGeo, new THREE.MeshStandardMaterial({ color: 0x059669, roughness: 0.5 }));
  rampMesh.rotation.x = -0.22;
  rampMesh.position.set(0, 2.5, 0);
  rampMesh.castShadow = true;
  rampGroup.add(rampMesh);

  rampGroup.position.set(x, 0, z);
  scene.add(rampGroup);
  obstacles.push(rampGroup);
}

/**
 * Removes all dynamic obstacles from the scene and resets the array.
 */
function clearObstacles() {
  obstacles.forEach(obj => scene.remove(obj));
  obstacles = [];
}

/**
 * Loads predefined arena obstacle layouts ('OPEN', 'CORRIDOR', 'DEADEND', 'OVERHANG_COURSE').
 * @param {string} presetKey - Layout preset identifier
 */
function loadArenaPreset(presetKey) {
  clearObstacles();
  resetVehicle();

  if (presetKey === 'CORRIDOR') {
    for (let z = -20; z <= 30; z += 10) {
      spawnObstacle(-12, z, 4, 8, 8);
      spawnObstacle(12, z, 4, 8, 8);
    }
  } else if (presetKey === 'DEADEND') {
    spawnObstacle(0, 25, 24, 8, 4);
    spawnObstacle(-10, 10, 4, 8, 20);
    spawnObstacle(10, 10, 4, 8, 20);
  } else if (presetKey === 'OVERHANG_COURSE') {
    spawnOverhangBridge(0, 18, true);
    spawnGroundBump(0, -10);
    spawnObstacle(-15, 25, 6, 8, 6);
    spawnObstacle(15, 25, 6, 8, 6);
  }
}

/* ==========================================================================
   6. USER CONTROLS & HUD ACTIONS
   ========================================================================== */

/**
 * Resets the vehicle position, heading, velocity, and rotation back to origin.
 */
function resetVehicle() {
  vehicleState.x = 0;
  vehicleState.y = 0;
  vehicleState.z = 0;
  vehicleState.heading = 0;
  vehicleState.pitch = 0;
  vehicleState.roll = 0;
  vehicleState.linearVelocity = 0;
  vehicleState.angularVelocity = 0;
  carGroup.position.set(0, 0, 0);
  carGroup.rotation.set(0, 0, 0);
}

/**
 * Toggles control mode between autonomous logic and manual WASD driving.
 * @param {'AUTO'|'MANUAL'} mode - Target operating mode
 */
function setControlMode(mode) {
  controlMode = mode;
  const btnAuto = document.getElementById('btn-mode-auto');
  const btnManual = document.getElementById('btn-mode-manual');

  if (mode === 'AUTO') {
    btnAuto.className = "btn-mode active";
    btnManual.className = "btn-mode";
  } else {
    btnManual.className = "btn-mode active";
    btnAuto.className = "btn-mode";
  }
}

/** Forces a specific 0-7 truth-table state for testing */
function forceTruthState(index) { forcedStateIndex = index; }

/** Clears forced override and returns to live 3D sensor raycasting */
function clearForcedState() { forcedStateIndex = null; }

/**
 * Updates physics simulation speed multiplier from the slider
 * @param {string|number} val - Multiplier value
 */
function updateSimSpeed(val) {
  simSpeedMultiplier = parseFloat(val);
  setElText('speed-multiplier-val', `${simSpeedMultiplier.toFixed(1)}x`);
}

/**
 * Toggles third-person smooth chase camera follow mode.
 */
function toggleCameraFollow() {
  isCameraFollow = !isCameraFollow;
  const btn = document.getElementById('btn-cam-toggle');
  btn.className = isCameraFollow ? "btn-utility active" : "btn-utility";
}

/* ==========================================================================
   7. DIGITAL LOGIC IC PIPELINE & TELEMETRY
   ========================================================================== */

/**
 * Evaluates live 3D sensor raycasts against obstacles:
 * - Multi-pitch Front ultrasonic rays (center, up, down)
 * - Angled Left and Right IR proximity sensor rays
 * Looks up the matched hardware state from the TRUTH_TABLE and updates the UI.
 * @returns {Object} Matched hardware state record
 */
function evaluateHardwareLogicPipeline() {
  let F = 0, L = 0, R = 0;

  if (forcedStateIndex !== null) {
    const forced = TRUTH_TABLE[forcedStateIndex];
    F = forced.F; L = forced.L; R = forced.R;
  } else {
    const carPos = carGroup.position;
    const carRot = carGroup.rotation;

    const forwardVec = new THREE.Vector3(0, 0, 1).applyEuler(carRot).normalize();
    const upVec = new THREE.Vector3(0, 1, 0).applyEuler(carRot).normalize();
    const rightVec = new THREE.Vector3(1, 0, 0).applyEuler(carRot).normalize();

    // Front Ultrasonic Multi-Pitch Sampling
    const sensorOrigin = new THREE.Vector3().copy(carPos).addScaledVector(upVec, 3.2).addScaledVector(forwardVec, 9.0);
    
    const centerRay = forwardVec.clone();
    const upRay = forwardVec.clone().addScaledVector(upVec, 0.28).normalize();
    const downRay = forwardVec.clone().addScaledVector(upVec, -0.18).normalize();

    const frontBeams = [centerRay, upRay, downRay];
    let frontHitDetected = false;

    for (let i = 0; i < frontBeams.length; i++) {
      raycaster.set(sensorOrigin, frontBeams[i]);
      raycaster.far = FRONT_THRESHOLD;
      const hits = raycaster.intersectObjects(obstacles, true);
      if (hits.length > 0 && hits[0].distance <= FRONT_THRESHOLD) {
        frontHitDetected = true;
        break;
      }
    }

    if (frontHitDetected) {
      F = 1;
      frontConeMesh.material.color.setHex(0xef4444);
      frontUpRayMesh.material.color.setHex(0xef4444);
      frontDownRayMesh.material.color.setHex(0xef4444);
    } else {
      frontConeMesh.material.color.setHex(0x22c55e);
      frontUpRayMesh.material.color.setHex(0x38bdf8);
      frontDownRayMesh.material.color.setHex(0x38bdf8);
    }

    // Left IR Proximity Raycasting
    const leftOrigin = new THREE.Vector3().copy(carPos).addScaledVector(upVec, 2.8).addScaledVector(rightVec, -4.5).addScaledVector(forwardVec, 7.5);
    const leftDir = forwardVec.clone().addScaledVector(rightVec, -0.577).normalize();
    raycaster.set(leftOrigin, leftDir);
    raycaster.far = SIDE_THRESHOLD;
    const leftHits = raycaster.intersectObjects(obstacles, true);

    if (leftHits.length > 0 && leftHits[0].distance <= SIDE_THRESHOLD) {
      L = 1;
      leftLineMesh.material.color.setHex(0xef4444);
    } else {
      leftLineMesh.material.color.setHex(0x22c55e);
    }

    // Right IR Proximity Raycasting
    const rightOrigin = new THREE.Vector3().copy(carPos).addScaledVector(upVec, 2.8).addScaledVector(rightVec, 4.5).addScaledVector(forwardVec, 7.5);
    const rightDir = forwardVec.clone().addScaledVector(rightVec, 0.577).normalize();
    raycaster.set(rightOrigin, rightDir);
    raycaster.far = SIDE_THRESHOLD;
    const rightHits = raycaster.intersectObjects(obstacles, true);

    if (rightHits.length > 0 && rightHits[0].distance <= SIDE_THRESHOLD) {
      R = 1;
      rightLineMesh.material.color.setHex(0xef4444);
    } else {
      rightLineMesh.material.color.setHex(0x22c55e);
    }
  }

  // Bitwise Encoding: (F << 2) | (L << 1) | R
  const stateIndex = (F << 2) | (L << 1) | R;
  const stateHardware = TRUTH_TABLE[stateIndex];

  updateTelemetryUI(F, L, R, stateHardware);
  return stateHardware;
}

/**
 * Updates HUD elements, IC logic badges, PWM bar, and ESP32 GPIO status rails.
 */
function updateTelemetryUI(F, L, R, state) {
  // Sensor Indicators
  const setSigUI = (id, val, name) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.innerText = `${name}: ${val}`;
    el.className = val ? "sensor-badge active" : "sensor-badge";
  };
  setSigUI('sig-front', F, 'F');
  setSigUI('sig-left', L, 'L');
  setSigUI('sig-right', R, 'R');

  // Digital IC Pipeline Display
  setElText('logic-dff-state', `${state.q} (${state.demux})`);
  setElText('logic-adder-count', `${state.count} (${F + L + R} Obs)`);
  setElText('logic-mux-speed', state.pwmText);
  setElText('logic-demux-dir', state.demux);

  // Header & Kinematics Badges
  setElText('header-action-badge', state.action.toUpperCase());
  setElText('telemetry-speed-val', `${state.pwmText} PWM`);
  
  const progressBar = document.getElementById('speed-progress-bar');
  if (progressBar) progressBar.style.width = `${state.pwm * 100}%`;
  
  setElText('truth-state-badge', `${state.stateName}: [F:${F}, L:${L}, R:${R}] -> ${state.demux.toUpperCase()}`);

  // ESP32 GPIO Rail Pins
  const setGPIO = (id, isHigh, labelHigh = "HIGH", labelLow = "LOW") => {
    const el = document.getElementById(id);
    if (!el) return;
    el.innerText = isHigh ? labelHigh : labelLow;
    el.className = isHigh ? "gpio-badge high" : "gpio-badge low";
  };

  setGPIO('gpio-5', true);
  setGPIO('gpio-18', F);
  setGPIO('gpio-19', L);
  setGPIO('gpio-21', R);
  setGPIO('gpio-34', state.q[0] === '1');
  setGPIO('gpio-35', state.q[1] === '1');

  const isFwd = state.demux === 'Forward';
  const isLeft = state.demux === 'Turn Left';
  const isRight = state.demux === 'Turn Right';
  const isStop = state.demux === 'Stop';

  setGPIO('gpio-22', (isFwd || isRight) && !isStop);
  setGPIO('gpio-23', isLeft && !isStop);
  setGPIO('gpio-25', (isFwd || isLeft) && !isStop);
  setGPIO('gpio-26', isRight && !isStop);

  const pwmVal = Math.round(state.pwm * 255);
  const g16 = document.getElementById('gpio-16');
  const g17 = document.getElementById('gpio-17');
  if (g16) g16.innerText = `${pwmVal} PWM`;
  if (g17) g17.innerText = `${pwmVal} PWM`;
}

/* ==========================================================================
   8. KINEMATICS, COLLISION & PHYSICS ENGINE
   ========================================================================== */

/**
 * Updates vehicle position, differential kinematics, collision bounding box
 * resolution with 30cm visual safety buffer, and wheel animations.
 * @param {number} dt - Elapsed frame delta time in seconds
 * @param {Object} hardwareState - Current IC truth table state
 */
function updatePhysics(dt, hardwareState) {
  let maxSpeed = 16.0 * simSpeedMultiplier;
  let turnRate = 2.4 * simSpeedMultiplier;

  if (controlMode === 'AUTO') {
    const speedFactor = hardwareState.pwm;

    if (hardwareState.demux === 'Forward') {
      vehicleState.targetSpeed = maxSpeed * speedFactor;
      vehicleState.targetTurn = 0;
    } else if (hardwareState.demux === 'Turn Left') {
      vehicleState.targetSpeed = maxSpeed * speedFactor * 0.45;
      vehicleState.targetTurn = turnRate;
    } else if (hardwareState.demux === 'Turn Right') {
      vehicleState.targetSpeed = maxSpeed * speedFactor * 0.45;
      vehicleState.targetTurn = -turnRate;
    } else {
      vehicleState.targetSpeed = 0;
      vehicleState.targetTurn = 0;
    }
  } else {
    let manualSpeed = 0;
    let manualTurn = 0;

    if (keysPressed['w']) manualSpeed += maxSpeed;
    if (keysPressed['s']) manualSpeed -= maxSpeed * 0.5;
    if (keysPressed['a']) manualTurn += turnRate;
    if (keysPressed['d']) manualTurn -= turnRate;

    vehicleState.targetSpeed = manualSpeed;
    vehicleState.targetTurn = manualTurn;
  }

  // Smooth Acceleration & Yaw Inertia Integration
  vehicleState.linearVelocity += (vehicleState.targetSpeed - vehicleState.linearVelocity) * 0.12;
  vehicleState.angularVelocity += (vehicleState.targetTurn - vehicleState.angularVelocity) * 0.18;

  vehicleState.heading += vehicleState.angularVelocity * dt;

  let moveDist = vehicleState.linearVelocity * dt;
  let moveDir = new THREE.Vector3(Math.sin(vehicleState.heading), 0, Math.cos(vehicleState.heading));
  let intendedPos = new THREE.Vector3().copy(carGroup.position).addScaledVector(moveDir, moveDist);

  // 30cm (3.0 scene units) visual safety buffer
  const BASE_CAR_RADIUS = 6.2;
  const SAFETY_BUFFER_30CM = 3.0;
  const CAR_RADIUS = BASE_CAR_RADIUS + SAFETY_BUFFER_30CM; // 9.2 units total
  const ARENA_BOUND = 63.0 - CAR_RADIUS;

  // Solid Mesh AABB Collision Solver
  for (let i = 0; i < obstacles.length; i++) {
    const obs = obstacles[i];
    const obsBox = new THREE.Box3().setFromObject(obs);

    const expandedMin = obsBox.min.clone().subScalar(CAR_RADIUS);
    const expandedMax = obsBox.max.clone().addScalar(CAR_RADIUS);

    if (intendedPos.y + 5.0 >= obsBox.min.y && intendedPos.y <= obsBox.max.y + 1.0) {
      if (intendedPos.x >= expandedMin.x && intendedPos.x <= expandedMax.x &&
          intendedPos.z >= expandedMin.z && intendedPos.z <= expandedMax.z) {
        
        const dx1 = intendedPos.x - expandedMin.x;
        const dx2 = expandedMax.x - intendedPos.x;
        const dz1 = intendedPos.z - expandedMin.z;
        const dz2 = expandedMax.z - intendedPos.z;

        const minPush = Math.min(dx1, dx2, dz1, dz2);

        if (minPush === dx1) intendedPos.x = expandedMin.x;
        else if (minPush === dx2) intendedPos.x = expandedMax.x;
        else if (minPush === dz1) intendedPos.z = expandedMin.z;
        else if (minPush === dz2) intendedPos.z = expandedMax.z;

        vehicleState.linearVelocity *= 0.15;
      }
    }
  }

  // Outer Arena Wall Bounds Constraining
  if (Math.abs(intendedPos.x) > ARENA_BOUND) {
    intendedPos.x = Math.sign(intendedPos.x) * ARENA_BOUND;
    vehicleState.linearVelocity *= 0.2;
  }
  if (Math.abs(intendedPos.z) > ARENA_BOUND) {
    intendedPos.z = Math.sign(intendedPos.z) * ARENA_BOUND;
    vehicleState.linearVelocity *= 0.2;
  }

  vehicleState.y = 0;
  vehicleState.pitch = 0;
  vehicleState.roll = 0;

  carGroup.position.set(intendedPos.x, 0, intendedPos.z);
  carGroup.rotation.set(0, vehicleState.heading, 0, 'YXZ');

  // Realistic Wheel Rotations
  const wheelRotSpeed = (vehicleState.linearVelocity / 2.0) * dt;
  leftWheel.children[0].rotation.x += wheelRotSpeed + vehicleState.angularVelocity * dt;
  rightWheel.children[0].rotation.x += wheelRotSpeed - vehicleState.angularVelocity * dt;
  casterWheel.rotation.x += wheelRotSpeed;

  // Update Kinematics HUD
  setElText('stat-speed-units', `${vehicleState.linearVelocity.toFixed(2)} <span class="text-[10px] text-slate-400">u/s</span>`);
  setElText('stat-steer-angle', `${(vehicleState.heading * (180 / Math.PI) % 360).toFixed(1)}°`);
  setElText('stat-elevation-y', `0.0u`);
}

/* ==========================================================================
   9. MAIN RAF ANIMATION LOOP & LIFECYCLE
   ========================================================================== */

let lastTime = performance.now();

/**
 * Main animation loop triggered every animation frame:
 * Computes delta time, updates hardware IC pipeline and kinematics,
 * handles camera follow lerping, and renders the scene.
 */
function animateLoop(now) {
  requestAnimationFrame(animateLoop);

  const dt = Math.min((now - lastTime) / 1000, 0.1);
  lastTime = now;

  const hardwareState = evaluateHardwareLogicPipeline();
  updatePhysics(dt, hardwareState);

  if (isCameraFollow) {
    const targetCamPos = new THREE.Vector3()
      .copy(carGroup.position)
      .sub(new THREE.Vector3(Math.sin(vehicleState.heading) * 36, -22, Math.cos(vehicleState.heading) * 36));
    camera.position.lerp(targetCamPos, 0.05);
    controls.target.lerp(carGroup.position, 0.05);
  }

  controls.update();
  renderer.render(scene, camera);
}

/**
 * Handles browser window resize and adjusts camera aspect ratio and renderer viewport.
 */
function onWindowResize() {
  const container = document.getElementById('canvas-container');
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

/**
 * Document load entry point
 */
window.onload = function() {
  initEngine();
};

