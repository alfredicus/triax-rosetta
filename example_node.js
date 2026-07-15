#!/usr/bin/env node
/**
 * triax (triaxial DEM simulation) — Node.js binding demo.
 *
 * Node counterpart of example_python.py. Exercises the rosetta-generated
 * `triax` N-API addon: the small math/geometry types (Vec3, Quaternion, Box),
 * the JSON parameter reader, the neighbor search, and finally a real (short)
 * triaxial compression test on the dense 8788-particle packing shipped with
 * the triax repo.
 *
 * Prerequisite — build the node-expanded addon first (stock compiler, no fork):
 *
 *     cd bindings/node-expanded && npm install && npx cmake-js rebuild
 *
 * Notes on the current binding surface (node-expanded backend):
 *   * Bound methods carry NO default arguments (rosetta does not capture C++
 *     default values), so every parameter must be passed explicitly.
 *   * NeighborSearch.findNeighbors(std::vector<Particle>&, ...) is NOT bound:
 *     the N-API backend skips methods whose parameter is a mutable, non-const
 *     reference (the marshalled argument is a temporary that cannot bind to a
 *     `T&`). Section 4 therefore resolves overlaps directly with the box's
 *     periodic distance — the same check the Python demo applies to the
 *     candidate list. Quaternion.toRotationMatrix(double&...) is skipped for
 *     the same reason.
 *   * Simulation is driven exactly like the `triax` executable: parameters come
 *     from a JSON file and the start configuration from CONF0/conf<name>,
 *     resolved relative to the current working directory; outputs are written
 *     to CONF/ and SUIVI/.
 */

"use strict";

const fs = require("fs");
const path = require("path");

const ROOT = __dirname;

// The compiled addon lives in bindings/node-expanded/ (next to its .node).
const triax = require(path.join(ROOT, "bindings", "node-expanded", "triax.node"));

function banner(n, title) {
    console.log();
    console.log("=".repeat(100));
    console.log(`${n}. ${title}`);
    console.log("=".repeat(100));
}

function vec(v) {
    return `(${v.x.toFixed(4)}, ${v.y.toFixed(4)}, ${v.z.toFixed(4)})`;
}

// f-string style helpers: ":.4f" and ":+.2f"
const f4 = (x) => x.toFixed(4);
const fp2 = (x) => (x >= 0 ? "+" : "") + x.toFixed(2);

function main() {
    banner(1, "Core math types: Vec3, Quaternion");

    const a = new triax.Vec3(1.0, 2.0, 3.0);
    const b = new triax.Vec3(4.0, 5.0, 6.0);
    console.log(`   a = ${vec(a)}  |a| = ${f4(a.norm())}`);
    console.log(`   a.dot(b)   = ${f4(a.dot(b))}`);
    console.log(`   a.cross(b) = ${vec(a.cross(b))}`);
    console.log(`   a.normalized() = ${vec(a.normalized())}`);

    // Quaternion(q0 scalar, q1..q3 vector): 90-degree rotation about z.
    const q = new triax.Quaternion(
        Math.cos(Math.PI / 4), 0.0, 0.0, Math.sin(Math.PI / 4));
    const r = q.transformToBody(new triax.Vec3(1.0, 0.0, 0.0));
    console.log(`   90-deg z-rotation of x-axis (to body frame): ${vec(r)}  |q| = ${f4(q.norm())}`);

    banner(2, "Periodic simulation box");

    // A 10x10x10 box. Positions are held in REDUCED coordinates s = r/h in
    // [-0.5, 0.5]; the box applies the periodic wrap and scales to real units.
    const box = new triax.Box();
    box.hl1 = new triax.Vec3(10.0, 10.0, 10.0);
    box.update();

    const p1 = new triax.Vec3(-0.48, 0.0, 0.0);  // near the -x face
    const p2 = new triax.Vec3(+0.48, 0.0, 0.0);  // near the +x face
    console.log(`   p1 (reduced) = ${vec(p1)} -> real ${vec(box.reducedToReal(p1))}`);
    console.log(`   p2 (reduced) = ${vec(p2)} -> real ${vec(box.reducedToReal(p2))}`);
    console.log(`   periodic distance p1-p2 = ${f4(box.computeDistance(p1, p2))}`
        + `   (0.4, across the boundary — not ${(9.6).toFixed(1)})`);
    console.log(`   Box.applyPBC(0.63) = ${fp2(triax.Box.applyPBC(0.63))}`);

    banner(3, "JsonParams: flat-JSON parameter reader");

    const params = triax.JsonParams.fromString(
        '{"friction": 0.35, "nom": "af1", "verbose": true, "itmax": 200000}'
    );
    console.log(`   friction = ${params.num("friction", 0.0)}`);
    console.log(`   nom      = ${params.str("nom", "?")}`);
    console.log(`   verbose  = ${params.boolean("verbose", false)}`);
    console.log(`   itmax    = ${params.integer("itmax", 0)}`);
    console.log(`   has('missing') = ${params.has("missing")}`
        + ` -> falls back: ${params.num("missing", -1.0)}`);

    banner(4, "Particles + neighbor overlap (periodic distance)");

    // 27 particles of radius 1.3 on a 3x3x3 lattice (spacing 2.5 in the
    // 10-box), so nearest lattice neighbors overlap slightly.
    const coords = [-0.25, 0.0, 0.25];
    const particles = [];
    for (const x of coords)
        for (const y of coords)
            for (const z of coords) {
                const p = new triax.Particle();
                p.s = new triax.Vec3(x, y, z);
                p.radius = 1.3;
                p.computeMass();
                particles.push(p);
            }

    // NeighborSearch.findNeighbors is not bound here (mutable vector<Particle>&
    // parameter — see the header note), so we resolve overlaps directly with
    // the box's periodic distance, exactly like the Python demo's `touching`
    // filter. `initialize` sets the cutoff the way the search would.
    const search = new triax.NeighborSearch();
    search.initialize(3.0, 1.3);   // cutoff distance, smallest radius

    const touching = [];
    for (let i = 0; i < particles.length; i++)
        for (let j = i + 1; j < particles.length; j++) {
            const d = box.computeDistance(particles[i].s, particles[j].s);
            const sumRadii = particles[i].radius + particles[j].radius;
            if (d < sumRadii) touching.push({ i, j, d, sumRadii });
        }

    console.log(`   ${particles.length} particles, cutoff ${search.cutoffDistance.toFixed(1)}`
        + ` -> ${touching.length} overlapping pairs (direct periodic-distance test)`);
    const pr = touching[0];
    console.log(`   first overlap: particles ${pr.i}-${pr.j}, distance ${pr.d.toFixed(2)}`
        + ` < sum of radii ${pr.sumRadii.toFixed(2)}`);

    banner(5, "Simulation: short triaxial compression test (8788 particles)");

    // Start from the dense packing (with its contact-force network) shipped in
    // the triax repo, and drive it like the `triax` executable would: stage the
    // config as CONF0/conf<name> in a scratch dir and point at a params file.
    const packing = path.join(ROOT, "extern", "triax", "examples",
        "Triax_vs_Rockable", "confpp1");
    const runDir = path.join(ROOT, "build", "node-demo");
    for (const sub of ["CONF0", "CONF", "SUIVI"])
        fs.mkdirSync(path.join(runDir, sub), { recursive: true });
    fs.copyFileSync(packing, path.join(runDir, "CONF0", "confpp1"));

    // A real test runs to max_strain ~0.1; keep it at 0.001 (~2 min) here.
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

    process.chdir(runDir);
    const sim = new triax.Simulation();
    sim.setInputFile("triax.json");
    console.log(`   running in ${runDir} ...`);
    sim.run();

    const s = sim.internalStress, t = sim.targetStress;
    console.log();
    console.log(`   iterations           = ${sim.iterationsDone()}`);
    console.log(`   axial strain (de3)   = ${sim.de3.toFixed(6)}   lateral: ${sim.de1.toFixed(6)}, ${sim.de2.toFixed(6)}`);
    console.log(`   internal stress      = ${vec(s)}`);
    console.log(`   lateral target       = ${f4(t.x)}  (servo holds sigma_1, sigma_2 there)`);
    console.log(`   deviator q = s3 - s1 = ${f4(s.z - s.x)}`);
    console.log(`   contacts             = ${sim.numContacts}  (sliding: ${sim.numSliding})`);
    console.log(`   final compacity      = ${f4(sim.finalCompacity())}`);
    console.log(`   box dimensions       = ${vec(sim.box.hl1)}`);
    console.log();
    console.log(`   outputs: ${runDir}/CONF/confdemo***  and  ${runDir}/SUIVI/suividemo`);
}

main();
