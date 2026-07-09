#!/usr/bin/env python3
"""triax (triaxial DEM simulation) — Python binding demo.

Exercises the rosetta-generated `triax` module: the small math/geometry types
(Vec3, Quaternion, Box), the JSON parameter reader, the Verlet neighbor
search, and finally a real (short) triaxial compression test on the dense
8788-particle packing shipped with the triax repo.

Prerequisite — build the optimized python module first:

    cmake -S bindings/python -B bindings/python/build -DCMAKE_BUILD_TYPE=Release
    cmake --build bindings/python/build -j12

(Release matters: the simulation in section 5 is ~20x slower unoptimized.)

Notes on the current binding surface:
  * Bound methods carry NO default arguments (rosetta does not capture C++
    default values), so every parameter must be passed explicitly.
  * Simulation is driven exactly like the `triax` executable: parameters come
    from a JSON file and the start configuration from CONF0/conf<name>,
    resolved relative to the current working directory; outputs are written
    to CONF/ and SUIVI/.
"""

import itertools
import math
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))

# The compiled `triax` module lives in bindings/python/ (next to its .so).
sys.path.insert(0, os.path.join(ROOT, "bindings", "python"))

import triax


def banner(n, title):
    print()
    print("=" * 100)
    print(f"{n}. {title}")
    print("=" * 100)


def vec(v):
    return f"({v.x:.4f}, {v.y:.4f}, {v.z:.4f})"


def main():
    banner(1, "Core math types: Vec3, Quaternion")

    a = triax.Vec3(1.0, 2.0, 3.0)
    b = triax.Vec3(4.0, 5.0, 6.0)
    print(f"   a = {vec(a)}  |a| = {a.norm():.4f}")
    print(f"   a.dot(b)   = {a.dot(b):.4f}")
    print(f"   a.cross(b) = {vec(a.cross(b))}")
    print(f"   a.normalized() = {vec(a.normalized())}")

    # Quaternion(q0 scalar, q1..q3 vector): 90-degree rotation about z.
    q = triax.Quaternion(math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    r = q.transformToBody(triax.Vec3(1.0, 0.0, 0.0))
    print(f"   90-deg z-rotation of x-axis (to body frame): {vec(r)}  |q| = {q.norm():.4f}")

    banner(2, "Periodic simulation box")

    # A 10x10x10 box. Positions are held in REDUCED coordinates s = r/h in
    # [-0.5, 0.5]; the box applies the periodic wrap and scales to real units.
    box = triax.Box()
    box.hl1 = triax.Vec3(10.0, 10.0, 10.0)
    box.update()

    p1 = triax.Vec3(-0.48, 0.0, 0.0)  # near the -x face
    p2 = triax.Vec3(+0.48, 0.0, 0.0)  # near the +x face
    print(f"   p1 (reduced) = {vec(p1)} -> real {vec(box.reducedToReal(p1))}")
    print(f"   p2 (reduced) = {vec(p2)} -> real {vec(box.reducedToReal(p2))}")
    print(f"   periodic distance p1-p2 = {box.computeDistance(p1, p2):.4f}"
          f"   (0.4, across the boundary — not {9.6:.1f})")
    print(f"   Box.applyPBC(0.63) = {triax.Box.applyPBC(0.63):+.2f}")

    banner(3, "JsonParams: flat-JSON parameter reader")

    params = triax.JsonParams.fromString(
        '{"friction": 0.35, "nom": "af1", "verbose": true, "itmax": 200000}'
    )
    print(f"   friction = {params.num('friction', 0.0)}")
    print(f"   nom      = {params.str('nom', '?')}")
    print(f"   verbose  = {params.boolean('verbose', False)}")
    print(f"   itmax    = {params.integer('itmax', 0)}")
    print(f"   has('missing') = {params.has('missing')}"
          f" -> falls back: {params.num('missing', -1.0)}")

    banner(4, "Particles + Verlet neighbor search")

    # 27 particles of radius 1.3 on a 3x3x3 lattice (spacing 2.5 in the
    # 10-box), so nearest lattice neighbors overlap slightly.
    particles = []
    for x, y, z in itertools.product([-0.25, 0.0, 0.25], repeat=3):
        p = triax.Particle()
        p.s = triax.Vec3(x, y, z)
        p.radius = 1.3
        p.computeMass()
        particles.append(p)

    search = triax.NeighborSearch()
    search.initialize(3.0, 1.3)              # cutoff distance, smallest radius
    search.findNeighbors(particles, 0, box, 0.9)  # particles, nLargeGrains, box, damping

    # findNeighbors builds the CANDIDATE list; actual overlap (pair.inContact)
    # is only resolved by the Simulation force loop, so check it ourselves
    # with the box's periodic distance.
    pairs = search.neighborList
    touching = [pr for pr in pairs
                if box.computeDistance(particles[pr.i].s, particles[pr.j].s) < pr.sumRadii]
    print(f"   {len(particles)} particles, cutoff 3.0 -> {len(pairs)} candidate pairs,"
          f" {len(touching)} overlapping")
    pr = touching[0]
    d = box.computeDistance(particles[pr.i].s, particles[pr.j].s)
    print(f"   first overlap: particles {pr.i}-{pr.j}, distance {d:.2f}"
          f" < sum of radii {pr.sumRadii:.2f}")

    banner(5, "Simulation: short triaxial compression test (8788 particles)")

    # Start from the dense packing (with its contact-force network) shipped in
    # the triax repo, and drive it like the `triax` executable would: stage the
    # config as CONF0/conf<name> in a scratch dir and point at a params file.
    packing = os.path.join(ROOT, "extern", "triax", "examples",
                           "Triax_vs_Rockable", "confpp1")
    run_dir = os.path.join(ROOT, "build", "python-demo")
    for sub in ("CONF0", "CONF", "SUIVI"):
        os.makedirs(os.path.join(run_dir, sub), exist_ok=True)
    shutil.copy(packing, os.path.join(run_dir, "CONF0", "confpp1"))

    # A real test runs to max_strain ~0.1; keep it at 0.001 (~2 min) here.
    with open(os.path.join(run_dir, "triax.json"), "w") as f:
        f.write("""{
            "nom_init": "pp1", "nom_ref": "pp1", "nom_radii": "pp1",
            "nom_out": "demo",
            "friction": 0.3,
            "lateral_stress": 0.1,
            "axial_strain_rate": 1e-4,
            "damping": 0.9,
            "max_strain": 0.001,
            "display_interval": 2e-4,
            "verbose": true
        }""")

    os.chdir(run_dir)
    sim = triax.Simulation()
    sim.setInputFile("triax.json")
    print(f"   running in {run_dir} ...")
    sim.run()

    s, t = sim.internalStress, sim.targetStress
    print()
    print(f"   iterations           = {sim.iterationsDone()}")
    print(f"   axial strain (de3)   = {sim.de3:.6f}   lateral: {sim.de1:.6f}, {sim.de2:.6f}")
    print(f"   internal stress      = {vec(s)}")
    print(f"   lateral target       = {t.x:.4f}  (servo holds sigma_1, sigma_2 there)")
    print(f"   deviator q = s3 - s1 = {s.z - s.x:.4f}")
    print(f"   contacts             = {sim.numContacts}  (sliding: {sim.numSliding})")
    print(f"   final compacity      = {sim.finalCompacity():.4f}")
    print(f"   box dimensions       = {vec(sim.box.hl1)}")
    print()
    print(f"   outputs: {run_dir}/CONF/confdemo***  and  {run_dir}/SUIVI/suividemo")


if __name__ == "__main__":
    main()
