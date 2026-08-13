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

// -- Frequency-dependent propagation -----------------------------------------------------------
// The scalar transmission above is broadband; sound passing THROUGH a partition is in reality
// low-pass filtered (a partition's transmission loss rises with frequency - mass law, ~6 dB per
// octave), which is why a gunshot in the next room sounds MUFFLED, not merely quieter. These
// per-band helpers carry that spectrum so cross-room bleed can be darkened, not just attenuated.

// Fills `outBands` with a material's frequency-dependent transmission: the authored broadband
// `transmission` scalar anchors the mid band (500 Hz) and a mass-law-like tilt lets low frequencies
// pass MORE and high frequencies LESS through the surface. A fully-open material (transmission 1)
// stays flat at 1; a perfect blocker (0) stays 0. Monotonic in the scalar.
void MaterialTransmissionBands(const AcousticMaterial& m, float outBands[kNumBands]);

// Per-band portal transmission: fully closed (openness 0) is the closed material's per-band curve;
// fully open (1) passes every band; linear per band in between. An air gap passes highs and lows
// equally, so a cracked-open door is brighter than the wall around it.
void PortalTransmissionBands(const AcousticMaterial& closedMaterial, float openness,
                             float outBands[kNumBands]);

// Per-band CombineTransmission: `outBands` (starting implicitly at all-1) becomes the product of
// each surface's per-band transmission curve. `surfaceBands` is `count` x kNumBands, row-major
// (surface s, band b -> surfaceBands[s*kNumBands + b]). Empty/null path = fully open (all 1).
void CombineTransmissionBands(const float* surfaceBands, int count, float outBands[kNumBands]);

// -- Room-to-room propagation graph ------------------------------------------------------------
// Acoustic coupling between rooms is rarely a single straight line: sound reaches a distant room by
// the BEST path through the graph of rooms connected by portals/walls (e.g. A -> B -> listener when
// the direct A -> listener wall blocks more than two doorways in series). These types let a host
// describe that graph; SolvePropagation finds each room's per-band coupling to the listener's room.

// Bounds for one propagation solve (allocation-free; a host may declare fewer).
static const int kMaxPropagationRooms = 64;
static const int kMaxPropagationEdges = 256;

// An undirected acoustic link between two rooms (indices into the caller's room array). It carries
// the per-band fraction of energy that passes between them (a portal opening and/or the wall
// material) AND the propagation `distance` in metres between them. Coupling reciprocity makes the
// link undirected; the distance adds geometric spreading + air absorption so a farther room couples
// less and darker, independent of the walls. `distance` defaults to 0 (no distance attenuation) so
// a transmission-only caller keeps the pure-transmission behavior; `transmission` must be filled.
// Standard-layout + trivially copyable (safe to pass as a C array across the ABI).
struct PropagationEdge {
    int roomA;
    int roomB;
    float transmission[kNumBands];
    float distance = 0.0f;
};

// Solves each room's per-band acoustic coupling to the listener's room by max-product relaxation
// over the edge graph: coupling[room][band] is the product, along the BEST (least-attenuating) path
// to `listenerRoom`, of each edge's per-band transmission TIMES its distance attenuation (an
// inverse-distance geometric rolloff beyond a reference range, plus per-band air absorption that
// darkens the highs over distance). So a nearby room through a doorway couples more than a far one
// through the same doorway, and a two-hop path can beat a shorter but more-blocked direct wall. The
// listener room is coupled 1 in every band; unreachable rooms are 0. Edges are undirected (sound
// flows both ways). `couplingOut` is roomCount*kNumBands, row-major (room r, band b ->
// couplingOut[r*kNumBands + b]). Allocation-free, O(roomCount * edgeCount * kNumBands). Out-of-range
// indices/counts are ignored safely.
void SolvePropagation(int roomCount, const PropagationEdge* edges, int edgeCount, int listenerRoom,
                      float* couplingOut);

// -- Diffraction -------------------------------------------------------------------------------
// When the direct path to a source is occluded but an opening offers a clearer route, the sound
// should appear to come from the OPENING, not through the wall. This is the geometry-free MODEL: the
// host supplies the source, listener, direct-path transmission, and the candidate openings with
// their transmissions (measured with its own geometry queries); the library owns the decision.

// A candidate opening for diffraction. `position` is world space; `openness` in [0,1]; `srcToPortal`
// and `portalToListener` are the [0,1] transmissions along those two legs (host-measured).
struct DiffractionPortal {
    float position[3];
    float openness;
    float srcToPortal;
    float portalToListener;
};

// Writes into `outPos` the apparent source position: pulled toward the best clearer opening when the
// direct path is occluded, or the true source when the direct path is clear (transmission high) or no
// opening helps. Position-only: a wrong guess re-angles a voice but never silences it. The pull
// strength grows as the direct path gets more blocked. `directTransmission` is the source->listener
// transmission; openings below an internal openness threshold, or no clearer than the direct path,
// are ignored. `outPos` may not alias `src`.
void DiffractedSourcePosition(const float src[3], const float listener[3], float directTransmission,
                              const DiffractionPortal* portals, int portalCount, float outPos[3]);

// Headless self-test of the model (materials, room acoustics, propagation). Returns true on pass;
// prints failures to stderr. Used by the library's tests and by consumer test harnesses.
bool SelfTest();

} // namespace hdsr

#endif // HDSR_ACOUSTICS_H_
