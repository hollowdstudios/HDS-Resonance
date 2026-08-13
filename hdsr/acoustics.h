// hdsr/acoustics.h - HDS-Resonance engine-agnostic acoustics library (public API).
//
// This is the HDS-Resonance acoustics MODEL layer: a standalone, engine-agnostic library of
// game-acoustics primitives - acoustic materials (frequency-dependent absorption + scattering +
// transmission), shoebox room acoustics (early-reflection coefficients + RT60 with air
// absorption), and acoustic propagation (transmission through surfaces, portal openings). It sits
// beside the vraudio binaural RENDERER (which HDS-Resonance also provides) and depends on NEITHER a
// game engine NOR a physics engine: geometry and ray/query RESULTS are supplied by the integrating
// engine, and this layer turns them into acoustic parameters that drive the renderer.
//
// Any engine (Heartbreak is the first) integrates by translating its own materials/scene into these
// types and feeding the results to the renderer; the library owns the acoustics, the engine owns
// the geometry/physics/ECS/device. C++11, header-only-friendly POD types (C-array fields), no
// dynamic allocation in the hot path.
//
// Copyright 2026 Hollow Dream Studios. Licensed under the Apache License, Version 2.0 (matching the
// upstream Resonance Audio fork this ships within).
#ifndef HDSR_ACOUSTICS_H_
#define HDSR_ACOUSTICS_H_

#include <cstddef>

namespace hdsr {

// Octave-band count for frequency-dependent acoustic quantities. Bands are centred at
// 31.25, 62.5, 125, 250, 500, 1k, 2k, 4k, 8k Hz - matching the Resonance reverb bands so a
// material's absorption maps 1:1 onto the renderer's room reverb.
static const int kNumBands = 9;

// Frequency-dependent acoustic properties of a surface material. POD by design (C-array fields,
// no constructor) so it is trivially copyable and C-ABI-friendly. Use DefaultMaterial() for a
// sensible neutral value.
struct AcousticMaterial {
    // Fraction of incident sound energy ABSORBED per octave band: 0 = perfect reflector,
    // 1 = fully absorbed (no reflection).
    float absorption[kNumBands];
    // Diffuse-vs-specular reflection: 0 = mirror-like, 1 = fully diffuse.
    float scattering;
    // Broadband fraction of energy passing THROUGH the surface: 0 = perfect blocker, 1 =
    // acoustically transparent. Drives occlusion / cross-room transmission. (This is the concept
    // the stock Resonance material model lacks; HDS-Resonance adds it here.)
    float transmission;
};

// -- Material library --------------------------------------------------------------------------
// A built-in, named library of common architectural + environmental acoustic materials. Consumers
// may also author AcousticMaterial values directly; this is the convenient default set.
int MaterialCount();
const char* MaterialNameAt(int index);           // name of preset [0, MaterialCount())
const AcousticMaterial* MaterialAt(int index);    // material of preset, or null if out of range
const AcousticMaterial* FindMaterial(const char* name); // exact-name lookup, or null
const AcousticMaterial& DefaultMaterial();        // a generic hard-ish solid surface (index 0)

// Reflection coefficient derived from a material's mid-band (500/1k/2k Hz) absorption:
// sqrt(1 - avg_absorption). 0 = fully absorbed, 1 = perfect reflector.
float ReflectionCoefficient(const AcousticMaterial& m);

// -- Room acoustics ----------------------------------------------------------------------------
// Early-reflection + late-reverb parameters of a shoebox room, ready to drive a renderer.
struct RoomAcoustics {
    // Per-wall reflection coefficient, world axes: [0]-x [1]+x [2]-y(floor) [3]+y(ceiling)
    // [4]-z [5]+z. 0 = fully absorbed, 1 = perfect reflector.
    float reflectionCoefficients[6];
    float cutoffFrequencyHz;     // low-pass -3 dB cutoff applied to early reflections
    float reflectionGain;        // uniform gain on early reflections
    float rt60[kNumBands];       // reverberation time per octave band (seconds)
    float reverbGain;            // late-reverb tail level
};

// Computes a room's acoustics from its shoebox dimensions + per-surface materials, using Eyring's
// reverberation equation WITH air absorption (so large rooms have a darker, shorter high-frequency
// tail). `dimensions` is the full box size in metres. `walls` are 6 materials in the reflection-
// coefficient order above (null entries fall back to DefaultMaterial()). `reflectionGain`/
// `reverbGain` scale the two effects; `reverbTimeScale` scales all RT60s (1 = no change).
RoomAcoustics ComputeRoomAcoustics(const float dimensions[3],
                                   const AcousticMaterial* const walls[6], float reflectionGain,
                                   float reverbGain, float reverbTimeScale);

// -- Propagation -------------------------------------------------------------------------------
// Total fraction of energy transmitted through a sequence of `count` surfaces, given their
// individual transmission coefficients (their product): 1 = clear path, 0 = fully blocked.
float CombineTransmission(const float* transmissions, int count);

// Transmission of a portal/opening: fully closed (openness 0) uses the closed material's
// transmission; fully open (1) passes everything; linear in between.
float PortalTransmission(const AcousticMaterial& closedMaterial, float openness);

// Headless self-test of the model (materials, room acoustics, propagation). Returns true on pass;
// prints failures to stderr. Used by the library's tests and by consumer test harnesses.
bool SelfTest();

} // namespace hdsr

#endif // HDSR_ACOUSTICS_H_
