#!/usr/bin/env node
/**
 * triax (triaxial DEM simulation) — WebAssembly binding demo.
 *
 * WebAssembly counterpart of example_python.py / example_node.js. Exercises the
 * rosetta-generated `triax` embind module: the small math/geometry types (Vec3,
 * Quaternion, Box), the JSON parameter reader, the neighbor search, and finally
 * a real (short) triaxial compression test on the dense 8788-particle packing
 * shipped with the triax repo.
 *
 * Prerequisite — build the wasm module with a stock emsdk (no fork). Section 5
 * runs a FILE-BASED simulation, so the module must be built with filesystem
 * access exported; pass the extra linker flags at configure time (the generated
 * CMakeLists stays untouched):
 *
 *     cd bindings/wasm-expanded
 *     emcmake cmake -S . -B build \
 *       -DCMAKE_EXE_LINKER_FLAGS="-sEXPORTED_RUNTIME_METHODS=FS,NODEFS -sFORCE_FILESYSTEM=1 -lnodefs.js"
 *     cmake --build build
 *
 * Notes on the current binding surface (wasm-expanded backend):
 *   * The module is asynchronous: `createModule()` returns a promise resolving
 *     to the instantiated module `M`; every bound type hangs off `M`.
 *   * Bound methods carry NO default arguments (rosetta does not capture C++
 *     default values), so every parameter must be passed explicitly.
 *   * NeighborSearch.findNeighbors(std::vector<Particle>&, ...) is NOT bound:
 *     the embind backend skips methods whose parameter is a mutable, non-const
 *     reference (the marshalled argument is a temporary that cannot bind to a
 *     `T&`). Section 4 therefore resolves overlaps directly with the box's
 *     periodic distance — the same check the Python demo applies to the
 *     candidate list. Quaternion.toRotationMatrix(double&...) is skipped for
 *     the same reason.
 *   * Section 5 mounts the run directory into the module's virtual filesystem
 *     with NODEFS, so the C++ `fopen` paths resolve to real files on disk, and
 *     drives the Simulation exactly like the `triax` executable: parameters from
 *     a JSON file, start config from CONF0/conf<name>, outputs to CONF/ SUIVI/.
 */

"use strict";

const fs = require("fs");
const path = require("path");

const ROOT = __dirname;

// The compiled embind module lives in bindings/wasm-expanded/build/.
const createModule = require(
    path.join(ROOT, "bindings", "wasm-expanded", "build", "triax.js"));

function banner(n, title) {
    console.log();
    console.log("=".repeat(100));
    console.log(`${n}. ${title}`);
    console.log("=".repeat(100));
}

function vec(v) {
    return `(${v.x.toFixed(4)}, ${v.y.toFixed(4)}, ${v.z.toFixed(4)})`;
}

const f4 = (x) => x.toFixed(4);
const fp2 = (x) => (x >= 0 ? "+" : "") + x.toFixed(2);

async function main() {
    const M = await createModule();

    banner(1, "Core math types: Vec3, Quaternion");

    const a = new M.Vec3(1.0, 2.0, 3.0);
    const b = new M.Vec3(4.0, 5.0, 6.0);
    console.log(`   a = ${vec(a)}  |a| = ${f4(a.norm())}`);
    console.log(`   a.dot(b)   = ${f4(a.dot(b))}`);
    const axb = a.cross(b);
    console.log(`   a.cross(b) = ${vec(axb)}`);
    const an = a.normalized();
    console.log(`   a.normalized() = ${vec(an)}`);

    // Quaternion(q0 scalar, q1..q3 vector): 90-degree rotation about z.
    const q = new M.Quaternion(
        Math.cos(Math.PI / 4), 0.0, 0.0, Math.sin(Math.PI / 4));
    const xaxis = new M.Vec3(1.0, 0.0, 0.0);
    const r = q.transformToBody(xaxis);
    console.log(`   90-deg z-rotation of x-axis (to body frame): ${vec(r)}  |q| = ${f4(q.norm())}`);

    // embind uses manual memory management — release the temporaries.
    [a, b, axb, an, q, xaxis, r].forEach((o) => o.delete());

    banner(2, "Periodic simulation box");

    // A 10x10x10 box. Positions are held in REDUCED coordinates s = r/h in
    // [-0.5, 0.5]; the box applies the periodic wrap and scales to real units.
    const box = new M.Box();
    const hl1 = new M.Vec3(10.0, 10.0, 10.0);
    box.hl1 = hl1;
    box.update();

    const p1 = new M.Vec3(-0.48, 0.0, 0.0);  // near the -x face
    const p2 = new M.Vec3(+0.48, 0.0, 0.0);  // near the +x face
    const p1r = box.reducedToReal(p1), p2r = box.reducedToReal(p2);
    console.log(`   p1 (reduced) = ${vec(p1)} -> real ${vec(p1r)}`);
    console.log(`   p2 (reduced) = ${vec(p2)} -> real ${vec(p2r)}`);
    console.log(`   periodic distance p1-p2 = ${f4(box.computeDistance(p1, p2))}`
        + `   (0.4, across the boundary — not ${(9.6).toFixed(1)})`);
    console.log(`   Box.applyPBC(0.63) = ${fp2(M.Box.applyPBC(0.63))}`);
    [hl1, p1, p2, p1r, p2r].forEach((o) => o.delete());

    banner(3, "JsonParams: flat-JSON parameter reader");

    const params = M.JsonParams.fromString(
        '{"friction": 0.35, "nom": "af1", "verbose": true, "itmax": 200000}'
    );
    console.log(`   friction = ${params.num("friction", 0.0)}`);
    console.log(`   nom      = ${params.str("nom", "?")}`);
    console.log(`   verbose  = ${params.boolean("verbose", false)}`);
    console.log(`   itmax    = ${params.integer("itmax", 0)}`);
    console.log(`   has('missing') = ${params.has("missing")}`
        + ` -> falls back: ${params.num("missing", -1.0)}`);
    params.delete();

    banner(4, "Particles + neighbor overlap (periodic distance)");

    // 27 particles of radius 1.3 on a 3x3x3 lattice (spacing 2.5 in the
    // 10-box), so nearest lattice neighbors overlap slightly.
    const coords = [-0.25, 0.0, 0.25];
    const particles = [];
    for (const x of coords)
        for (const y of coords)
            for (const z of coords) {
                const p = new M.Particle();
                const s = new M.Vec3(x, y, z);
                p.s = s;
                p.radius = 1.3;
                p.computeMass();
                s.delete();
                particles.push(p);
            }

    // NeighborSearch.findNeighbors is not bound here (mutable vector<Particle>&
    // parameter — see the header note), so we resolve overlaps directly with
    // the box's periodic distance, exactly like the Python demo's `touching`
    // filter. `initialize` sets the cutoff the way the search would.
    const search = new M.NeighborSearch();
    search.initialize(3.0, 1.3);   // cutoff distance, smallest radius

    const touching = [];
    for (let i = 0; i < particles.length; i++)
        for (let j = i + 1; j < particles.length; j++) {
            const si = particles[i].s, sj = particles[j].s;
            const d = box.computeDistance(si, sj);
            const sumRadii = particles[i].radius + particles[j].radius;
            if (d < sumRadii) touching.push({ i, j, d, sumRadii });
            si.delete(); sj.delete();
        }

    console.log(`   ${particles.length} particles, cutoff ${search.cutoffDistance.toFixed(1)}`
        + ` -> ${touching.length} overlapping pairs (direct periodic-distance test)`);
    const pr = touching[0];
    console.log(`   first overlap: particles ${pr.i}-${pr.j}, distance ${pr.d.toFixed(2)}`
        + ` < sum of radii ${pr.sumRadii.toFixed(2)}`);
    search.delete();
    particles.forEach((p) => p.delete());

    banner(5, "Simulation: short triaxial compression test (8788 particles)");

    // Start from the dense packing (with its contact-force network) shipped in
    // the triax repo, and drive it like the `triax` executable would: stage the
    // config as CONF0/conf<name> in a scratch dir and point at a params file.
    const packing = path.join(ROOT, "extern", "triax", "examples",
        "Triax_vs_Rockable", "confpp1");
    const runDir = path.join(ROOT, "build", "wasm-demo");
    for (const sub of ["CONF0", "CONF", "SUIVI"])
        fs.mkdirSync(path.join(runDir, sub), { recursive: true });
    fs.copyFileSync(packing, path.join(runDir, "CONF0", "confpp1"));

    // A real test runs to max_strain ~0.1; keep it at 0.001 (~a few min) here.
    fs.writeFileSync(path.join(runDir, "triax.json"), `{
        "nom_init": "pp1", "nom_ref": "pp1", "nom_radii": "pp1",
        "nom_out": "demo",
        "friction": 0.3,
        "lateral_stress": 0.1,
        "axial_strain_rate": 1e-4,
        "damping": 0.9,
        "max_strain": 0.001,
        "display_interval": 2e-4,
        "verbose": true
    }`);

    // Mount the real run directory into the module's virtual FS via NODEFS, then
    // make it the C++ working directory — the simulation's fopen()s now hit the
    // real files on disk. (This is the process.chdir(run_dir) of the Node demo.)
    M.FS.mkdir("/work");
    M.FS.mount(M.NODEFS, { root: runDir }, "/work");
    M.FS.chdir("/work");

    const sim = new M.Simulation();
    sim.setInputFile("triax.json");
    console.log(`   running in ${runDir} ...`);
    sim.run();

    const s = sim.internalStress, t = sim.targetStress, bx = sim.box;
    console.log();
    console.log(`   iterations           = ${sim.iterationsDone()}`);
    console.log(`   axial strain (de3)   = ${sim.de3.toFixed(6)}   lateral: ${sim.de1.toFixed(6)}, ${sim.de2.toFixed(6)}`);
    console.log(`   internal stress      = ${vec(s)}`);
    console.log(`   lateral target       = ${f4(t.x)}  (servo holds sigma_1, sigma_2 there)`);
    console.log(`   deviator q = s3 - s1 = ${f4(s.z - s.x)}`);
    console.log(`   contacts             = ${sim.numContacts}  (sliding: ${sim.numSliding})`);
    console.log(`   final compacity      = ${f4(sim.finalCompacity())}`);
    console.log(`   box dimensions       = ${vec(bx.hl1)}`);
    console.log();
    console.log(`   outputs: ${runDir}/CONF/confdemo***  and  ${runDir}/SUIVI/suividemo`);

    const bxhl = bx.hl1;
    [s, t, bxhl, bx, sim, box].forEach((o) => o.delete());
}

main().catch((e) => { console.error(e); process.exit(1); });
