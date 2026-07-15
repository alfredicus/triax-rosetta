/*
 * triax WebAssembly demo — Web Worker.
 *
 * Runs the rosetta-generated embind module off the main thread so the page stays
 * responsive and the (long, blocking) Section-5 simulation can be interrupted by
 * terminating this worker. Every line the C++ prints (printf/std::cout) flows
 * through Emscripten's print/printErr; we forward those — plus our own JS log
 * lines — to the page with postMessage, so the page's console panel shows
 * EVERYTHING in one place.
 *
 * Message protocol (worker -> page):
 *   {type:'log',    stream:'out'|'err'|'js', text}
 *   {type:'status', text, kind:''|'ok'|'err'}
 *   {type:'ready'}                      module up, sections 1-4 printed
 *   {type:'runStarted'} / {type:'runDone'} / {type:'runError', message}
 * Message protocol (page -> worker): {cmd:'run'}
 */
"use strict";

// The module + its wasm live under bindings/wasm-expanded/build/. importScripts
// resolves relative to this worker's URL (repo root); locateFile (below) pins the
// .wasm next to its .js regardless of where the worker itself sits.
const BUILD_DIR = "bindings/wasm-expanded/build/";
const PACKING_URL = "extern/triax/examples/Triax_vs_Rockable/confpp1";

importScripts(BUILD_DIR + "triax.js");   // defines self.createModule

// ---- forwarding helpers ----------------------------------------------------
const post = (m) => self.postMessage(m);
const log = (text, stream = "js") => post({ type: "log", stream, text });
const status = (text, kind = "") => post({ type: "status", text, kind });
const banner = (n, title) => {
  log("");
  log("=".repeat(92));
  log(`${n}. ${title}`);
  log("=".repeat(92));
};

// ---- formatting (mirrors example_wasm.js) ----------------------------------
const f4 = (x) => x.toFixed(4);
const fp2 = (x) => (x >= 0 ? "+" : "") + x.toFixed(2);
const vec = (v) => `(${v.x.toFixed(4)}, ${v.y.toFixed(4)}, ${v.z.toFixed(4)})`;

const PARAMS = `{
    "nom_init": "pp1", "nom_ref": "pp1", "nom_radii": "pp1",
    "nom_out": "demo",
    "friction": 0.3,
    "lateral_stress": 0.1,
    "axial_strain_rate": 1e-4,
    "damping": 0.9,
    "max_strain": 0.001,
    "display_interval": 2e-4,
    "verbose": true
}`;

let M = null;   // the instantiated module

// ---- sections 1–4 : pure in-memory -----------------------------------------
function runCoreSections() {
  banner(1, "Core math types: Vec3, Quaternion");
  {
    const a = new M.Vec3(1.0, 2.0, 3.0);
    const b = new M.Vec3(4.0, 5.0, 6.0);
    const axb = a.cross(b);
    const an = a.normalized();
    log(`   a = ${vec(a)}  |a| = ${f4(a.norm())}`);
    log(`   a.dot(b)   = ${f4(a.dot(b))}`);
    log(`   a.cross(b) = ${vec(axb)}`);
    log(`   a.normalized() = ${vec(an)}`);
    const q = new M.Quaternion(Math.cos(Math.PI / 4), 0.0, 0.0, Math.sin(Math.PI / 4));
    const xaxis = new M.Vec3(1.0, 0.0, 0.0);
    const r = q.transformToBody(xaxis);
    log(`   90-deg z-rotation of x-axis (to body frame): ${vec(r)}  |q| = ${f4(q.norm())}`);
    [a, b, axb, an, q, xaxis, r].forEach((o) => o.delete());
  }

  banner(2, "Periodic simulation box");
  {
    const box = new M.Box();
    const hl1 = new M.Vec3(10.0, 10.0, 10.0);
    box.hl1 = hl1; box.update();
    const p1 = new M.Vec3(-0.48, 0.0, 0.0);
    const p2 = new M.Vec3(+0.48, 0.0, 0.0);
    const p1r = box.reducedToReal(p1), p2r = box.reducedToReal(p2);
    log(`   p1 (reduced) = ${vec(p1)} -> real ${vec(p1r)}`);
    log(`   p2 (reduced) = ${vec(p2)} -> real ${vec(p2r)}`);
    log(`   periodic distance p1-p2 = ${f4(box.computeDistance(p1, p2))}   (0.4, across the boundary)`);
    log(`   Box.applyPBC(0.63) = ${fp2(M.Box.applyPBC(0.63))}`);
    [hl1, p1, p2, p1r, p2r, box].forEach((o) => o.delete());
  }

  banner(3, "JsonParams: flat-JSON parameter reader");
  {
    const params = M.JsonParams.fromString(
      '{"friction": 0.35, "nom": "af1", "verbose": true, "itmax": 200000}');
    log(`   friction = ${params.num("friction", 0.0)}`);
    log(`   nom      = ${params.str("nom", "?")}`);
    log(`   verbose  = ${params.boolean("verbose", false)}`);
    log(`   itmax    = ${params.integer("itmax", 0)}`);
    log(`   has('missing') = ${params.has("missing")} -> falls back: ${params.num("missing", -1.0)}`);
    params.delete();
  }

  banner(4, "Particles + neighbor overlap (periodic distance)");
  {
    const coords = [-0.25, 0.0, 0.25];
    const particles = [];
    for (const x of coords) for (const y of coords) for (const z of coords) {
      const p = new M.Particle();
      const s = new M.Vec3(x, y, z);
      p.s = s; p.radius = 1.3; p.computeMass();
      s.delete(); particles.push(p);
    }
    const box = new M.Box();
    const hl1 = new M.Vec3(10.0, 10.0, 10.0);
    box.hl1 = hl1; box.update();
    const search = new M.NeighborSearch();
    search.initialize(3.0, 1.3);

    const touching = [];
    for (let i = 0; i < particles.length; i++)
      for (let j = i + 1; j < particles.length; j++) {
        const si = particles[i].s, sj = particles[j].s;
        const d = box.computeDistance(si, sj);
        const sumRadii = particles[i].radius + particles[j].radius;
        if (d < sumRadii) touching.push({ i, j, d, sumRadii });
        si.delete(); sj.delete();
      }
    log(`   ${particles.length} particles, cutoff ${search.cutoffDistance.toFixed(1)} -> ` +
        `${touching.length} overlapping pairs (direct periodic-distance test)`);
    const pr = touching[0];
    log(`   first overlap: particles ${pr.i}-${pr.j}, distance ${pr.d.toFixed(2)} < sum of radii ${pr.sumRadii.toFixed(2)}`);
    search.delete();
    particles.forEach((p) => p.delete());
    [hl1, box].forEach((o) => o.delete());
  }
}

// ---- section 5 : file-based simulation via MEMFS ---------------------------
async function runSimulation() {
  banner(5, "Simulation: short triaxial compression test (8788 particles)");
  post({ type: "runStarted" });
  status("running simulation…", "");

  log("   fetching dense packing…");
  const resp = await fetch(PACKING_URL);
  if (!resp.ok)
    throw new Error(`fetch ${PACKING_URL} -> HTTP ${resp.status}. Serve the repo root over HTTP.`);
  const packing = new Uint8Array(await resp.arrayBuffer());

  // Stage the run tree in MEMFS (NODEFS is Node-only): CONF0/confpp1 is the start
  // config; CONF/ and SUIVI/ receive the outputs. Then chdir so fopen() paths hit it.
  const FS = M.FS;
  for (const d of ["/work", "/work/CONF0", "/work/CONF", "/work/SUIVI"]) {
    try { FS.mkdir(d); } catch (e) { /* EEXIST on re-run */ }
  }
  FS.writeFile("/work/CONF0/confpp1", packing);
  FS.writeFile("/work/triax.json", PARAMS);
  FS.chdir("/work");
  log("   staged CONF0/confpp1 + triax.json in MEMFS; running…");

  const sim = new M.Simulation();
  sim.setInputFile("triax.json");
  sim.run();   // blocks this worker (not the page); Stop = terminate this worker

  const s = sim.internalStress, tgt = sim.targetStress, bx = sim.box, bxhl = bx.hl1;
  log("");
  log(`   iterations           = ${sim.iterationsDone()}`);
  log(`   axial strain (de3)   = ${sim.de3.toFixed(6)}   lateral: ${sim.de1.toFixed(6)}, ${sim.de2.toFixed(6)}`);
  log(`   internal stress      = ${vec(s)}`);
  log(`   lateral target       = ${f4(tgt.x)}  (servo holds sigma_1, sigma_2 there)`);
  log(`   deviator q = s3 - s1 = ${f4(s.z - s.x)}`);
  log(`   contacts             = ${sim.numContacts}  (sliding: ${sim.numSliding})`);
  log(`   final compacity      = ${f4(sim.finalCompacity())}`);
  log(`   box dimensions       = ${vec(bxhl)}`);
  log("");
  log("   outputs written under /work/CONF/ and /work/SUIVI/ (MEMFS)");
  [s, tgt, bxhl, bx, sim].forEach((o) => o.delete());
}

// ---- page -> worker commands -----------------------------------------------
self.onmessage = async (e) => {
  if (!e.data || e.data.cmd !== "run") return;
  try {
    await runSimulation();
    post({ type: "runDone" });
    status("simulation finished", "ok");
  } catch (err) {
    const message = err && err.message ? err.message : String(err);
    log("   " + message, "err");
    post({ type: "runError", message });
    status("simulation failed", "err");
  }
};

// ---- boot ------------------------------------------------------------------
(async function boot() {
  try {
    status("instantiating module…", "");
    // print/printErr capture the C++ stdout/stderr; locateFile pins the .wasm.
    M = await createModule({
      locateFile: (p) => BUILD_DIR + p,
      print: (text) => log(text, "out"),
      printErr: (text) => log(text, "err"),
    });
    runCoreSections();
    status("module ready", "ok");
    post({ type: "ready" });
  } catch (err) {
    console.error(err);
    log("boot failed: " + (err && err.message ? err.message : String(err)), "err");
    status("failed to instantiate module", "err");
  }
})();
