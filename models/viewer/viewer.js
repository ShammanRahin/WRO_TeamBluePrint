/* ============================================================
   Team Blueprint — 3D Robot Assembly Viewer Engine
   Three.js + STLLoader interactive viewport
   ============================================================ */

(function () {
  'use strict';

  // --- State & Globals ---
  let scene, camera, renderer, controls;
  let gridHelper, axesHelper;
  let modelGroup; // Root container for all robot parts
  const loadedParts = new Map(); // filename -> { mesh, origPos, category, material }
  let totalTris = 0;

  // Material Palettes
  const PALETTES = {
    default: {
      body: { color: 0xe02838, roughness: 0.28, metalness: 0.12, clearcoat: 0.3 },
      chassis: { color: 0x181a20, roughness: 0.65, metalness: 0.2 },
      knuckles: { color: 0x2b2e38, roughness: 0.5, metalness: 0.3 },
      wheels: { color: 0xf5f6f8, roughness: 0.35, metalness: 0.05 },
      powertrain: { color: 0x222630, roughness: 0.6, metalness: 0.4 },
      mounts: { color: 0xc82030, roughness: 0.35, metalness: 0.1 }
    },
    stealth: {
      body: { color: 0x1a1c22, roughness: 0.4, metalness: 0.2 },
      chassis: { color: 0x0f1115, roughness: 0.8, metalness: 0.1 },
      knuckles: { color: 0x252830, roughness: 0.5, metalness: 0.4 },
      wheels: { color: 0x2a2e36, roughness: 0.6, metalness: 0.1 },
      powertrain: { color: 0x15181e, roughness: 0.7, metalness: 0.5 },
      mounts: { color: 0x1f222a, roughness: 0.5, metalness: 0.2 }
    },
    blueprint: {
      body: { color: 0x0066ff, roughness: 0.3, metalness: 0.2 },
      chassis: { color: 0x002266, roughness: 0.7, metalness: 0.3 },
      knuckles: { color: 0x0088ff, roughness: 0.4, metalness: 0.2 },
      wheels: { color: 0x66b2ff, roughness: 0.3, metalness: 0.1 },
      powertrain: { color: 0x003399, roughness: 0.5, metalness: 0.4 },
      mounts: { color: 0x0055dd, roughness: 0.4, metalness: 0.2 }
    },
    clay: {
      body: { color: 0xdcdfe4, roughness: 0.9, metalness: 0.0 },
      chassis: { color: 0xc4c7cd, roughness: 0.9, metalness: 0.0 },
      knuckles: { color: 0xb5b8bf, roughness: 0.9, metalness: 0.0 },
      wheels: { color: 0xe8ebf0, roughness: 0.9, metalness: 0.0 },
      powertrain: { color: 0xaaadb4, roughness: 0.9, metalness: 0.0 },
      mounts: { color: 0xd0d3d8, roughness: 0.9, metalness: 0.0 }
    }
  };

  let currentPaletteName = 'default';
  let isWireframe = false;
  let isAutoRotate = true;

  // Pre-configured Part List in models/
  const PART_DEFINITIONS = [
    { file: 'Base Plate.stl', category: 'chassis', explodeZ: -15 },
    { file: 'Middle Plate.stl', category: 'chassis', explodeZ: 15 },
    { file: 'Front Bumper.stl', category: 'body', explodeZ: 10, explodeY: -20 },
    { file: 'Tail .stl', category: 'body', explodeZ: 40, explodeY: 30 },
    { file: 'Left Stearing Hand.stl', category: 'knuckles', explodeZ: 0, explodeX: -25 },
    { file: 'Right Stearing Hand.stl', category: 'knuckles', explodeZ: 0, explodeX: 25 },
    { file: 'LEGO Gear BOX.stl', category: 'powertrain', explodeZ: 5 },
    { file: 'LEGO Shaft.stl', category: 'powertrain', explodeZ: 0 },
    { file: 'Motor Mount.stl', category: 'powertrain', explodeZ: 5 },
    { file: 'Connector B22.stl', category: 'chassis', explodeZ: 5 },
    { file: 'Left Front Wheel.stl', category: 'wheels', explodeX: -40 },
    { file: 'Right Front Wheel.stl', category: 'wheels', explodeX: 40 },
    { file: 'Right Rare Wheel.stl', category: 'wheels', explodeX: 40 },
    { file: 'Left Rare Wheel New.stl', category: 'wheels', explodeX: -40 },
    { file: 'Left BR.stl', category: 'powertrain', explodeX: -20 },
    { file: 'Center BR.stl', category: 'powertrain', explodeZ: -5 },
    { file: 'Right BR.stl', category: 'powertrain', explodeX: 20 },
    { file: 'LiDAR mount.stl', category: 'mounts', explodeZ: 35 },
    { file: 'Back TOF.stl', category: 'mounts', explodeZ: 10, explodeY: 25 }
  ];

  // --- Init Engine ---
  function init() {
    const container = document.getElementById('canvas-container');

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0c12);

    // Camera
    const aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(45, aspect, 1, 3000);
    camera.position.set(220, 180, 260);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.1;
    container.appendChild(renderer.domElement);

    // Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;
    controls.autoRotate = isAutoRotate;
    controls.autoRotateSpeed = 1.2;
    controls.maxPolarAngle = Math.PI / 2 + 0.1; // Don't flip under floor too far
    controls.minDistance = 30;
    controls.maxDistance = 1200;

    // Lights
    setupLighting();

    // Helpers
    gridHelper = new THREE.GridHelper(400, 40, 0xff3344, 0x1f2430);
    gridHelper.position.y = -40;
    scene.add(gridHelper);

    axesHelper = new THREE.AxesHelper(60);
    axesHelper.position.set(-150, -39, -150);
    scene.add(axesHelper);

    // Container for model meshes
    modelGroup = new THREE.Group();
    scene.add(modelGroup);

    // Events
    window.addEventListener('resize', onWindowResize);
    setupUI();
    setupDragAndDrop();

    // Start loading models
    loadDefaultAssembly();

    // Animation Loop
    animate();
  }

  // --- Lighting Setup ---
  function setupLighting() {
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x222630, 0.8);
    scene.add(hemiLight);

    const keyLight = new THREE.DirectionalLight(0xffffff, 1.4);
    keyLight.position.set(200, 300, 200);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x88bbff, 0.8);
    fillLight.position.set(-200, 150, -150);
    scene.add(fillLight);

    const rimLight = new THREE.DirectionalLight(0xff8866, 0.6);
    rimLight.position.set(0, -100, -200);
    scene.add(rimLight);
  }

  // --- Material Creator ---
  function getMaterial(category) {
    const pal = PALETTES[currentPaletteName] || PALETTES.default;
    const spec = pal[category] || pal.body;
    return new THREE.MeshStandardMaterial({
      color: spec.color,
      roughness: spec.roughness,
      metalness: spec.metalness,
      wireframe: isWireframe,
      side: THREE.DoubleSide
    });
  }

  // --- Model Loader ---
  function loadSTL(path, definition) {
    const loader = new THREE.STLLoader();
    return new Promise((resolve, reject) => {
      loader.load(
        path,
        (geometry) => {
          geometry.computeVertexNormals();
          geometry.center();

          const mat = getMaterial(definition.category);
          const mesh = new THREE.Mesh(geometry, mat);

          // Calculate faces/tris
          const count = geometry.attributes.position.count / 3;
          totalTris += count;

          // Store part data
          loadedParts.set(definition.file, {
            mesh: mesh,
            origPos: mesh.position.clone(),
            category: definition.category,
            explodeX: definition.explodeX || 0,
            explodeY: definition.explodeY || 0,
            explodeZ: definition.explodeZ || 0
          });

          modelGroup.add(mesh);
          resolve(mesh);
        },
        undefined,
        (err) => {
          // If relative path fails, silent resolve so other parts load
          console.warn(`Could not load STL directly via fetch (${path}):`, err);
          resolve(null);
        }
      );
    });
  }

  // Load default repository STLs
  async function loadDefaultAssembly() {
    setLoading(true, 'Loading CAD Components...');
    let loadedCount = 0;
    const total = PART_DEFINITIONS.length;

    for (let i = 0; i < total; i++) {
      const def = PART_DEFINITIONS[i];
      const relPath = `../${def.file}`;
      updateProgress((i / total) * 100, `Loading ${def.file}...`);
      const res = await loadSTL(relPath, def);
      if (res) loadedCount++;
    }

    updateMetadata();
    setLoading(false);

    // If fetch was blocked (e.g. file:// protocol without local server)
    if (loadedCount === 0) {
      showLocalServerNotice();
    } else {
      fitCameraToObject(modelGroup);
    }
  }

  // Fallback / notice if running directly on file:// without HTTP server
  function showLocalServerNotice() {
    const text = document.getElementById('loading-text');
    text.innerHTML = `Running offline via file:// protocol.<br/><span style="font-size:12px; color:#aaa;">Drop any .stl file from the models folder onto the screen to inspect!</span>`;
    document.getElementById('loading-overlay').classList.add('visible');
    setTimeout(() => {
      document.getElementById('loading-overlay').classList.remove('visible');
    }, 4000);
  }

  // --- UI Event Listeners ---
  function setupUI() {
    // Part Selector Dropdown
    const partSelect = document.getElementById('part-select');
    partSelect.addEventListener('change', (e) => {
      const target = e.target.value;
      if (target === 'ALL') {
        loadedParts.forEach(({ mesh }) => (mesh.visible = true));
        fitCameraToObject(modelGroup);
      } else {
        loadedParts.forEach(({ mesh }, name) => {
          mesh.visible = name === target;
          if (mesh.visible) fitCameraToObject(mesh);
        });
      }
    });

    // Wireframe Toggle
    const btnWireframe = document.getElementById('btn-wireframe');
    btnWireframe.addEventListener('click', () => {
      isWireframe = !isWireframe;
      btnWireframe.classList.toggle('active', isWireframe);
      updateMaterials();
    });

    // Auto-Rotate Toggle
    const btnRotate = document.getElementById('btn-autorotate');
    btnRotate.addEventListener('click', () => {
      isAutoRotate = !isAutoRotate;
      controls.autoRotate = isAutoRotate;
      btnRotate.classList.toggle('active', isAutoRotate);
    });

    // Grid / Axes Toggle
    const btnAxes = document.getElementById('btn-axes');
    btnAxes.addEventListener('click', () => {
      const visible = !gridHelper.visible;
      gridHelper.visible = visible;
      axesHelper.visible = visible;
      btnAxes.classList.toggle('active', visible);
    });

    // Reset Camera
    const btnReset = document.getElementById('btn-reset-view');
    btnReset.addEventListener('click', () => {
      fitCameraToObject(modelGroup);
    });

    // Exploded View Slider
    const explodeSlider = document.getElementById('explode-slider');
    const explodeVal = document.getElementById('explode-val');
    explodeSlider.addEventListener('input', (e) => {
      const factor = parseFloat(e.target.value) / 100.0;
      explodeVal.textContent = `${Math.round(factor * 100)}%`;
      applyExplosion(factor);
    });

    // Appearance Palette Picker
    const colorBtns = document.querySelectorAll('.color-btn');
    colorBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        colorBtns.forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        currentPaletteName = btn.getAttribute('data-color');
        updateMaterials();
      });
    });

    // File Input Upload
    const fileInput = document.getElementById('file-input');
    fileInput.addEventListener('change', (e) => {
      const files = e.target.files;
      if (files.length) handleUserFiles(files);
    });
  }

  // --- Exploded View Transform ---
  function applyExplosion(factor) {
    loadedParts.forEach(({ mesh, origPos, explodeX, explodeY, explodeZ }) => {
      mesh.position.x = origPos.x + (explodeX || 0) * factor * 1.5;
      mesh.position.y = origPos.y + (explodeY || 0) * factor * 1.5;
      mesh.position.z = origPos.z + (explodeZ || 0) * factor * 2.0;
    });
  }

  // --- Update Materials ---
  function updateMaterials() {
    loadedParts.forEach(({ mesh, category }) => {
      mesh.material = getMaterial(category);
    });
  }

  // --- Drag & Drop Handler ---
  function setupDragAndDrop() {
    window.addEventListener('dragover', (e) => {
      e.preventDefault();
      document.body.classList.add('drag-over');
    });

    window.addEventListener('dragleave', (e) => {
      if (e.clientX <= 0 || e.clientY <= 0) {
        document.body.classList.remove('drag-over');
      }
    });

    window.addEventListener('drop', (e) => {
      e.preventDefault();
      document.body.classList.remove('drag-over');
      const files = e.dataTransfer.files;
      if (files.length) handleUserFiles(files);
    });
  }

  // Process User Dropped Files
  function handleUserFiles(files) {
    setLoading(true, 'Parsing 3D File...');
    const loader = new THREE.STLLoader();

    Array.from(files).forEach((file) => {
      const reader = new FileReader();
      reader.onload = function (e) {
        try {
          const contents = e.target.result;
          const geometry = loader.parse(contents);
          geometry.computeVertexNormals();
          geometry.center();

          const mat = getMaterial('body');
          const mesh = new THREE.Mesh(geometry, mat);

          const count = geometry.attributes.position.count / 3;
          totalTris += count;

          loadedParts.set(file.name, {
            mesh: mesh,
            origPos: mesh.position.clone(),
            category: 'body',
            explodeZ: 0
          });

          modelGroup.add(mesh);
          fitCameraToObject(modelGroup);
          updateMetadata();

          // Add to select dropdown
          const opt = document.createElement('option');
          opt.value = file.name;
          opt.textContent = `📁 ${file.name}`;
          document.getElementById('part-select').appendChild(opt);
          document.getElementById('part-select').value = file.name;
        } catch (err) {
          console.error('Error parsing STL file:', err);
          alert('Could not parse 3D file: ' + file.name);
        } finally {
          setLoading(false);
        }
      };
      reader.readAsArrayBuffer(file);
    });
  }

  // --- Camera Auto-Fit ---
  function fitCameraToObject(obj) {
    const box = new THREE.Box3().setFromObject(obj);
    if (box.isEmpty()) return;

    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());

    const maxDim = Math.max(size.x, size.y, size.z);
    const fov = camera.fov * (Math.PI / 180);
    let cameraZ = Math.abs((maxDim / 2) / Math.tan(fov / 2));
    cameraZ *= 2.0;

    camera.position.set(center.x + cameraZ * 0.7, center.y + cameraZ * 0.5, center.z + cameraZ * 0.8);
    camera.lookAt(center);
    controls.target.copy(center);
    controls.update();
  }

  // --- UI Helpers ---
  function setLoading(visible, text) {
    const overlay = document.getElementById('loading-overlay');
    const loadingText = document.getElementById('loading-text');
    if (text) loadingText.textContent = text;
    overlay.classList.toggle('visible', visible);
  }

  function updateProgress(pct, text) {
    document.getElementById('progress-fill').style.width = `${pct}%`;
    if (text) document.getElementById('loading-text').textContent = text;
  }

  function updateMetadata() {
    document.getElementById('tris-count').textContent = `${totalTris.toLocaleString()} Triangles`;
    document.getElementById('parts-loaded').textContent = `${loadedParts.size} Parts Loaded`;
  }

  function onWindowResize() {
    const container = document.getElementById('canvas-container');
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  }

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }

  // Run on DOM ready
  window.addEventListener('DOMContentLoaded', init);
})();
