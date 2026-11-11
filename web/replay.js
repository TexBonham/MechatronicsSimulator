// 3D replay (VIE-001, VIE-015): bodies drawn from the facade's `geometry`, posed from recorded `pose:` series.
// Frames follow the design convention: z up, cylinder and capsule along the shape's z axis, box `size` the full
// edges, capsule `length` the cylindrical part, orientation quaternions w first. Values are placed as recorded;
// nothing is computed from them (FAC-016).
import * as THREE from "./vendor/three.module.js";

const POSE_FIELDS = ["x", "y", "z", "roll", "pitch", "yaw"];
const COLORS = [0x4e79a7, 0xf28e2b, 0x59a14f, 0xe15759, 0x76b7b2, 0xedc948, 0xb07aa1, 0xff9da7];

export function poseSeries(body) {
  return POSE_FIELDS.map((field) => `pose:${body}.${field}`);
}

function known(values) {
  if (values === null || values === undefined) {
    return false;
  }
  if (Array.isArray(values)) {
    return values.every((value) => typeof value === "number");
  }
  return typeof values === "number";
}

// The three.js geometry for one shape, or null when a size is still an open parameter.
export function shapeGeometry(shape) {
  const sizes = shape.sizes || {};
  if (shape.shape === "box" && known(sizes.size)) {
    return new THREE.BoxGeometry(sizes.size[0], sizes.size[1], sizes.size[2]);
  }
  if (shape.shape === "sphere" && known(sizes.radius)) {
    return new THREE.SphereGeometry(sizes.radius, 24, 16);
  }
  if (shape.shape === "cylinder" && known(sizes.radius) && known(sizes.length)) {
    const geometry = new THREE.CylinderGeometry(sizes.radius, sizes.radius, sizes.length, 24);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (shape.shape === "capsule" && known(sizes.radius) && known(sizes.length)) {
    const geometry = new THREE.CapsuleGeometry(sizes.radius, sizes.length, 8, 24);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (shape.shape === "plane") {
    let width = 10;
    let depth = 10;
    if (known(sizes.size)) {
      width = sizes.size[0];
      depth = sizes.size[1];
    }
    return new THREE.PlaneGeometry(width, depth);
  }
  if (Array.isArray(shape.vertices) && shape.vertices.length > 0) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(shape.vertices.flat(), 3));
    return geometry;
  }
  return null;
}

function shapeObject(shape, color) {
  const geometry = shapeGeometry(shape);
  if (geometry === null) {
    return null;
  }
  let object = null;
  if (Array.isArray(shape.vertices) && shape.vertices.length > 0) {
    object = new THREE.Points(geometry, new THREE.PointsMaterial({ color, size: 4, sizeAttenuation: false }));
  } else {
    const material = new THREE.MeshStandardMaterial({ color, side: THREE.DoubleSide });
    object = new THREE.Mesh(geometry, material);
  }
  if (known(shape.position)) {
    object.position.set(shape.position[0], shape.position[1], shape.position[2]);
  }
  if (known(shape.orientation) && shape.orientation.length === 4) {
    const [w, x, y, z] = shape.orientation;
    object.quaternion.set(x, y, z, w);
  }
  return object;
}

export class Replay {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf4f5f7);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
    this.camera.up.set(0, 0, 1);
    this.orbit = new THREE.Spherical(4, 1.1, 0.8);
    this.target = new THREE.Vector3(0, 0, 0);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x777788, 2.0));
    const sun = new THREE.DirectionalLight(0xffffff, 1.5);
    sun.position.set(3, -2, 5);
    this.scene.add(sun);
    const grid = new THREE.GridHelper(10, 20, 0xb0b4bb, 0xd5d8dd);
    grid.rotateX(Math.PI / 2);
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.5));
    this.bodies = new Map();
    this.attachControls();
    this.resize();
    window.addEventListener("resize", () => this.resize());
  }

  attachControls() {
    const element = this.renderer.domElement;
    let dragging = false;
    let lastX = 0;
    let lastY = 0;
    element.addEventListener("pointerdown", (event) => {
      dragging = true;
      lastX = event.clientX;
      lastY = event.clientY;
      element.setPointerCapture(event.pointerId);
    });
    element.addEventListener("pointerup", () => {
      dragging = false;
    });
    element.addEventListener("pointermove", (event) => {
      if (!dragging) {
        return;
      }
      this.orbit.theta -= (event.clientX - lastX) * 0.01;
      this.orbit.phi = THREE.MathUtils.clamp(this.orbit.phi - (event.clientY - lastY) * 0.01, 0.05, Math.PI - 0.05);
      lastX = event.clientX;
      lastY = event.clientY;
      this.render();
    });
    element.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        let factor = 1.1;
        if (event.deltaY < 0) {
          factor = 1 / 1.1;
        }
        this.orbit.radius = THREE.MathUtils.clamp(this.orbit.radius * factor, 0.05, 500);
        this.render();
      },
      { passive: false },
    );
  }

  resize() {
    const width = Math.max(this.container.clientWidth, 1);
    const height = Math.max(this.container.clientHeight, 1);
    this.renderer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  // `bodies` is the `geometry` result's list; returns the names of bodies with nothing drawable.
  load(bodies) {
    for (const group of this.bodies.values()) {
      this.scene.remove(group);
    }
    this.bodies.clear();
    const undrawn = [];
    bodies.forEach((body, index) => {
      const group = new THREE.Group();
      group.name = body.body;
      const color = COLORS[index % COLORS.length];
      for (const shape of body.shapes || []) {
        const object = shapeObject(shape, color);
        if (object !== null) {
          group.add(object);
        }
      }
      if (group.children.length === 0) {
        undrawn.push(body.body);
      }
      this.scene.add(group);
      this.bodies.set(body.body, group);
    });
    this.frame();
    return undrawn;
  }

  frame() {
    const box = new THREE.Box3();
    for (const group of this.bodies.values()) {
      box.expandByObject(group);
    }
    if (box.isEmpty()) {
      this.render();
      return;
    }
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    this.target.copy(sphere.center);
    this.orbit.radius = Math.max(sphere.radius * 3, 0.2);
    this.render();
  }

  // `pose` holds the recorded x, y, z, roll, pitch, yaw of one body at the cursor (rotation Z-Y-X).
  setPose(body, pose) {
    const group = this.bodies.get(body);
    if (group === undefined) {
      return;
    }
    group.position.set(pose.x, pose.y, pose.z);
    group.rotation.set(pose.roll, pose.pitch, pose.yaw, "ZYX");
  }

  render() {
    // Spherical is y-up; the scene is z-up, so its polar axis maps to z.
    const offset = new THREE.Vector3().setFromSpherical(this.orbit);
    this.camera.position.set(this.target.x + offset.x, this.target.y + offset.z, this.target.z + offset.y);
    this.camera.lookAt(this.target);
    this.renderer.render(this.scene, this.camera);
  }
}
