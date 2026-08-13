# HDS-Resonance — game acoustics/audio library

Status: **foundation started (2026-08-13).** HDS-Resonance is being developed as a **standalone,
engine-agnostic, open-source game audio/acoustics library** — comparable in spirit to a general
audio-acoustics middleware — with game engines as *clients*. It builds on the archived Google
Resonance Audio renderer (`vraudio`, Apache-2.0) and adds a modern acoustic **model** layer
(`hdsr`) on top.

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
- **`ReflectionCoefficient(material)`**, **`SelfTest()`** (headless library test).

Consumers translate their own materials → `hdsr::AcousticMaterial`, call these, and translate the
results into the renderer's room/source parameters. (Heartbreak: `.hbmat` acoustic fields ↔
`hdsr::AcousticMaterial`; `AcousticSpace`/`AcousticPortal` components → `ComputeRoomAcoustics` /
`PortalTransmission`; Jolt multi-hit rays → material transmissions → `CombineTransmission`.)

## Roadmap (the general-purpose feature set)

Implemented in `hdsr` now: acoustic materials, frequency-dependent absorption, scattering,
transmission, reflection coefficients, shoebox room acoustics, RT60, air absorption, occlusion
transmission, basic propagation, portal openings.

Planned (engine-agnostic, geometry/query results supplied by the engine):

- **Diffraction** — apparent-source repositioning toward openings, given portal + occlusion data
  (Heartbreak currently prototypes this engine-side; the *model* belongs here).
- **Portals/zones & multi-environment** — a room/portal graph + per-environment reverb so a
  source's *own* room tail is audible through a portal ("distant gunshot rings next door"). Likely
  a renderer-DSP extension for multiple concurrent reverb environments.
- **Geometry-based acoustic queries** — image-source early reflections, ray-traced occlusion/
  propagation over an engine-provided triangle/BVH interface (the engine supplies hits; `hdsr`
  supplies the model).
- **Reflection/reverb DSP** — richer, tunable reverb; frequency-dependent transmission filtering.
- **Dynamic acoustic scenes** — moving geometry/occluders, time-varying portals.
- **Offline/baked acoustic data** — precomputed room/propagation caches.
- **Full spatial-audio surface** — expose the renderer (HRTF/ambisonics/spatializer) through a
  clean library API alongside the model, so a consumer can adopt the whole stack.

## Build

`hdsr/acoustics.{h,cc}` compiles into `ResonanceAudioObj` → `ResonanceAudioStatic` (see
`resonance_audio/CMakeLists.txt` `RA_SOURCES`). The fork root is on the include path
(`CMakeLists.txt` `include_directories(${PROJECT_SOURCE_DIR})`), so consumers include
`"hdsr/acoustics.h"`. No new dependencies. Verified via Heartbreak's `--test-acoustics`
(runs `hdsr::SelfTest`).
