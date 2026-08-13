# HDS-Resonance — game acoustics/audio library

HDS-Resonance is a **standalone, engine-agnostic, open-source game audio/acoustics library** —
comparable in spirit to a general audio-acoustics middleware — with game engines as *clients*. It
builds on the archived Google Resonance Audio renderer (`vraudio`, Apache-2.0) and adds a modern
acoustic **model** layer (`hdsr`) on top. The model layer is implemented and headless-tested:
materials and frequency-dependent transmission, shoebox room acoustics, a room-to-room propagation
graph, multi-environment reverb (with per-band coupling, distance decay, pre-delay, and directional
pan), and diffraction.

Heartbreak Engine is the **first and deepest** integration target, but it is a *consumer*, not the
architect: the library API is designed for general games/engines, not around any one engine's
convenience.

```
                        HDS-RESONANCE  (this repo — the product)
                        engine-agnostic acoustics/audio library
                                        │
              ┌─────────────────────────┼─────────────────────────┐
              │                         │                         │
        Heartbreak Engine        Other engines            Standalone tools
        (first deep client)      (Unity-like, OSS)         (bakers, editors)
              │
              ▼  engine integration only:
        ECS translation · Jolt geometry/ray queries · .hbmat conversion · device/audio I/O
```

## Layers

- **`vraudio` (upstream renderer)** — binaural HRTF rendering, ambisonics, per-source spatializer,
  the single shoebox room reverb/reflection DSP. Kept faithful to upstream (see `MODIFICATIONS.md`).
- **`hdsr` (HDS acoustic model)** — the new, engine-agnostic acoustics layer this project adds. Pure
  acoustic math + data, **no engine or physics dependency**: geometry and ray/query *results* are
  supplied by the integrating engine, and `hdsr` turns them into acoustic parameters that drive the
  renderer. C++11, POD types (C-ABI-friendly), no hot-path allocation.

## The boundary (never crossed)

`hdsr` (and `vraudio`) must **never** depend on a game engine or physics engine — no engine headers,
namespaces, ECS, glm/entt, asset formats, or `ma_node`. Engines translate their own materials/scene
into the library's types and feed query results in. The library owns the *acoustics*; the engine
owns the *geometry/physics/ECS/device*. This is what lets Heartbreak, another engine, or a
standalone tool all integrate the same library.

## `hdsr/acoustics.h` — the acoustic model (implemented)

Engine-agnostic acoustic materials + room acoustics + propagation:

- **`AcousticMaterial`** — frequency-dependent `absorption[9]` (octave bands 31.25 Hz…8 kHz) +
  `scattering` + `transmission`. Transmission (energy passing *through* a surface — the basis of
  occlusion/cross-room bleed) is the concept the stock Resonance material model lacks; `hdsr` adds
  it. POD.
- **Material library** — `MaterialCount/MaterialNameAt/MaterialAt/FindMaterial/DefaultMaterial`: a
  built-in named set (Concrete, Brick, Wood, Glass, Metal, Drywall, Carpet, Curtain, Fiberglass,
  Grass, Water, Marble, Open/Air, …). This supersedes the stock 24-entry `MaterialName` enum for
  general use (richer, arbitrary materials, per-band data).
- **`RoomAcoustics ComputeRoomAcoustics(dims, walls[6], reflGain, reverbGain, reverbTimeScale)`** —
  per-wall reflection coefficients (`sqrt(1-absorption)`) + **Eyring RT60 with air absorption**
  (large rooms get a darker HF tail), from shoebox dimensions + per-face materials.
- **Propagation** — `CombineTransmission(t[], n)` (total transmission through a sequence of
  surfaces), `PortalTransmission(closedMaterial, openness)` (a door/window opening).
- **Frequency-dependent propagation** — `MaterialTransmissionBands` /
  `PortalTransmissionBands` / `CombineTransmissionBands`: the per-octave-band counterparts of the
  scalars above. A partition's transmission loss rises with frequency (mass law, ~6 dB/oct), so a
  material's broadband transmission is tilted into a curve that anchors the mid band and passes lows
  more than highs. This is what makes cross-room bleed *muffled*, not merely quieter.
- **Room-to-room propagation graph** — `PropagationEdge` + `SolvePropagation(roomCount, edges,
  edgeCount, listenerRoom, couplingOut)`: given rooms connected by portal/wall edges (each carrying
  a per-band transmission **and a distance**), a max-product relaxation finds each room's per-band
  coupling to the listener along the **best path** through the graph (A → B → listener when the
  direct A → listener wall blocks more than two doorways in series). Each edge's contribution is its
  transmission **times a distance attenuation** — an inverse-distance geometric rolloff beyond a
  reference range plus per-band air absorption that darkens the highs over distance — so a farther
  room couples less and darker, not just through thinner walls. Allocation-free, bounded.
- **Diffraction** — `DiffractionPortal` + `DiffractedSourcePosition(src, listener,
  directTransmission, portals, count, outPos)`: when the direct path to a source is occluded but an
  opening offers a clearer route, the apparent source is pulled toward the best opening (so the sound
  seems to come from the doorway, not through the wall). The host supplies the portals + per-leg
  transmissions from its own geometry queries; the library owns the decision. Position-only, so a
  wrong guess never silences a voice.
- **`ReflectionCoefficient(material)`**, **`SelfTest()`** (headless library test).

Consumers translate their own materials → `hdsr::AcousticMaterial`, call these, and translate the
results into the renderer's room/source parameters. (Heartbreak: `.hbmat` acoustic fields ↔
`hdsr::AcousticMaterial`; `AcousticSpace`/`AcousticPortal` components → `ComputeRoomAcoustics` /
`PortalTransmission`; Jolt multi-hit rays → material transmissions → `CombineTransmission`.)

## `hdsr/environment_reverb.h` — multi-environment reverb (implemented)

`EnvironmentReverb` — a stereo reverberator that runs **several acoustic environments at once**.
Each active environment (a room) has its own RT60 and contributes its **own** reverb tail, scaled
by how acoustically **coupled** it is to the listener (portal/propagation coupling supplied by the
host). The library **manages the set of environments** and **mixes ALL of them up to an explicit
capacity** — this is a performance cap, not a "pick the loudest room" selection; when over capacity
the *least-coupled* environment is dropped. Each environment persists its tail across blocks, so a
source in the next room keeps ringing there after it stops and leaks to the listener through the
coupling.

DSP: a per-environment feedback-delay network (8 delay lines, orthogonal Householder feedback,
per-line gain from RT60, per-line damping from the high-frequency RT60). Orthogonal + gain < 1 ⇒
unconditionally stable regardless of RT60s or environment count. Usage per audio block:
`BeginBlock()` → `SetEnvironment(id, rt60, coupling, gain)` per active room → `AddInput(id, mono, n)`
per source (routed into the room it occupies) → `Process(stereoOut, n)`. The host supplies **only**
room topology + coupling + source/listener state; the library owns the environments, the mixing,
and the reverb DSP. `SelfTest()` verifies impulse decay ≈ RT60, coupling scaling, and capacity
eviction. (Heartbreak: `AcousticWorld` declares every coupled `AcousticSpace` as an environment,
coupling = transmission from the room to the listener through portals; each spatial voice is routed
into its room; the tails mix into the binaural output inside the spatializer node.)

`coupling` is **per-band**: `SetEnvironment` has an overload taking a 9-band coupling curve (from
`SolvePropagation`). The environment's tail is mixed at the low-band coupling level *and* low-pass
filtered by the curve's high-to-low ratio, so a distant room reached through walls bleeds in
**darkened** (muffled), not merely quieter. A flat curve (the scalar overload) is the "just quieter"
special case — it applies no filtering, so the pre-existing scalar path is unchanged. `SelfTest()`
also checks that a darker-in-highs coupling reduces the tail's high-frequency content.

Each environment also has a **propagation pre-delay** (`SetEnvironmentPreDelay(id, seconds)`): its
reverb tail is delayed by `distance / speed-of-sound` before it mixes in, because the room's
reverberant field takes time to reach the listener. The listener's own room is ~0; a distant room's
tail arrives audibly later. It is a per-environment output delay line (the diffuse tail tolerates an
integer delay that changes as the listener moves); `0` is bit-identical to no pre-delay. `SelfTest()`
checks the tail onset shifts by the set delay.

Finally, each environment has a **stereo pan** (`SetEnvironmentPan(id, pan)`, −1…+1): the direction
its reverb arrives from. A room reached through a doorway off to one side bleeds in from *that* side
instead of sounding centred, so the reverb — not just the direct sound — helps localize the opening.
The listener's own room stays centred (0). It is a gentle bias (the tail keeps its stereo width),
smoothed as the listener turns. `SelfTest()` checks that a right-panned environment leans right and a
centred one stays balanced.

## Roadmap (the general-purpose feature set)

Implemented in `hdsr` now: acoustic materials, frequency-dependent absorption, scattering,
transmission, reflection coefficients, shoebox room acoustics, RT60, air absorption, occlusion
transmission, basic propagation, portal openings, **multi-environment reverb** (each active room
contributes its own coupling-weighted reverb tail, mixed up to an explicit capacity),
**frequency-dependent transmission** (mass-law-tilted per-band transmission through
materials/portals → muffled cross-room bleed), a **room-to-room propagation graph**
(`SolvePropagation` best-path per-band coupling), and **per-band environment coupling** (a distant
room's tail is darkened, not just quieter).

Also implemented: **diffraction** (`DiffractedSourcePosition` — apparent-source repositioning toward
the clearest opening when the direct path is occluded; the model now lives here, Heartbreak only
supplies the portal geometry + transmissions).

Planned (engine-agnostic, geometry/query results supplied by the engine):

- **Multi-environment refinement** — done: the reverb bank, per-band coupling, the room→room
  propagation graph, **distance/decay along edges** (geometric rolloff + per-band air absorption over
  distance), a **propagation pre-delay** (each room's tail arrives after `distance / c`), and a
  **directional pan** (each room's tail arrives from its own side) all exist. The graph is undirected
  because acoustic coupling is reciprocal. Remaining: reverb-tail EQ beyond the one-pole coupling
  filter, and audible tuning of the reference-distance / air-absorption / pre-delay constants.
- **Geometry-based acoustic queries** — image-source early reflections, ray-traced occlusion/
  propagation over an engine-provided triangle/BVH interface (the engine supplies hits; `hdsr`
  supplies the model).
- **Reflection/reverb DSP** — richer, tunable reverb; frequency-dependent transmission filtering.
- **Dynamic acoustic scenes** — moving geometry/occluders, time-varying portals.
- **Offline/baked acoustic data** — precomputed room/propagation caches.
- **Full spatial-audio surface** — expose the renderer (HRTF/ambisonics/spatializer) through a
  clean library API alongside the model, so a consumer can adopt the whole stack.

## Build

`hdsr/acoustics.{h,cc}` and `hdsr/environment_reverb.{h,cc}` compile into `ResonanceAudioObj` →
`ResonanceAudioStatic` (see `resonance_audio/CMakeLists.txt` `RA_SOURCES`). The fork root is on the
include path (`CMakeLists.txt` `include_directories(${PROJECT_SOURCE_DIR})`), so consumers include
`"hdsr/acoustics.h"` / `"hdsr/environment_reverb.h"`. No new dependencies. Verified via Heartbreak's
`--test-acoustics` (runs `hdsr::SelfTest` + `hdsr::EnvironmentReverb::SelfTest`).
