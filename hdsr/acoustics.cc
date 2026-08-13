// hdsr/acoustics.cc - see hdsr/acoustics.h.
//
// Copyright 2026 Hollow Dream Studios. Licensed under the Apache License, Version 2.0.
#include "hdsr/acoustics.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace hdsr {

namespace {

// Built-in acoustic material library. Absorption bands: 31.25, 62.5, 125, 250, 500, 1k, 2k, 4k,
// 8k Hz (drawn from common architectural-acoustics tables, extended at the extreme bands), with
// scattering + transmission. Transmission is broadband: higher = more sound passes through.
struct NamedMaterial {
    const char* name;
    AcousticMaterial material;
};

const NamedMaterial kMaterials[] = {
    {"Default", {{0.10f, 0.10f, 0.10f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f}, 0.20f, 0.12f}},
    // An opening (doorway/window gap): reflects nothing, passes everything.
    {"Open / Air", {{1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f}, 0.00f, 1.00f}},
    {"Concrete (sealed)",
     {{0.01f, 0.01f, 0.01f, 0.01f, 0.02f, 0.02f, 0.02f, 0.03f, 0.03f}, 0.12f, 0.04f}},
    {"Brick (bare)",
     {{0.02f, 0.02f, 0.03f, 0.03f, 0.03f, 0.04f, 0.05f, 0.07f, 0.07f}, 0.20f, 0.05f}},
    {"Wood Panel", {{0.19f, 0.19f, 0.28f, 0.22f, 0.17f, 0.09f, 0.10f, 0.11f, 0.11f}, 0.20f, 0.15f}},
    {"Wood Floor", {{0.10f, 0.10f, 0.15f, 0.11f, 0.10f, 0.07f, 0.06f, 0.07f, 0.07f}, 0.15f, 0.10f}},
    {"Glass (window)",
     {{0.30f, 0.30f, 0.35f, 0.25f, 0.18f, 0.12f, 0.07f, 0.05f, 0.05f}, 0.05f, 0.22f}},
    {"Glass (thick)",
     {{0.15f, 0.15f, 0.18f, 0.06f, 0.04f, 0.03f, 0.02f, 0.02f, 0.02f}, 0.05f, 0.10f}},
    {"Metal", {{0.02f, 0.02f, 0.02f, 0.03f, 0.03f, 0.03f, 0.04f, 0.04f, 0.05f}, 0.10f, 0.08f}},
    {"Drywall / Sheetrock",
     {{0.28f, 0.28f, 0.29f, 0.10f, 0.05f, 0.04f, 0.07f, 0.09f, 0.09f}, 0.15f, 0.35f}},
    {"Plaster (rough)",
     {{0.03f, 0.03f, 0.03f, 0.03f, 0.04f, 0.05f, 0.05f, 0.06f, 0.06f}, 0.25f, 0.08f}},
    {"Acoustic Ceiling Tile",
     {{0.40f, 0.45f, 0.50f, 0.55f, 0.65f, 0.75f, 0.80f, 0.85f, 0.85f}, 0.35f, 0.40f}},
    {"Heavy Curtain",
     {{0.10f, 0.12f, 0.14f, 0.35f, 0.55f, 0.72f, 0.70f, 0.65f, 0.65f}, 0.45f, 0.55f}},
    {"Carpet", {{0.02f, 0.03f, 0.03f, 0.06f, 0.14f, 0.37f, 0.60f, 0.65f, 0.65f}, 0.30f, 0.20f}},
    {"Fiberglass Insulation",
     {{0.20f, 0.35f, 0.50f, 0.70f, 0.85f, 0.95f, 0.98f, 0.98f, 0.98f}, 0.50f, 0.55f}},
    {"Grass / Soil",
     {{0.10f, 0.12f, 0.15f, 0.25f, 0.40f, 0.55f, 0.60f, 0.60f, 0.60f}, 0.55f, 0.15f}},
    {"Water Surface",
     {{0.01f, 0.01f, 0.01f, 0.01f, 0.02f, 0.02f, 0.02f, 0.03f, 0.03f}, 0.05f, 0.08f}},
    {"Marble / Polished Stone",
     {{0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.02f, 0.02f, 0.02f, 0.02f}, 0.05f, 0.04f}},
};
const int kMaterialCount = static_cast<int>(sizeof(kMaterials) / sizeof(kMaterials[0]));

// Air absorption per octave band (m, ~20 C / 50 % RH): negligible at low frequency, strong at
// high - it shortens the HF reverb tail, so large rooms read "darker".
const float kAir[kNumBands] = {0.0000f, 0.0000f, 0.0001f, 0.0003f, 0.0006f,
                               0.0010f, 0.0025f, 0.0080f, 0.0260f};

// Per-band exponents applied to a material's broadband transmission scalar t as t^exp: <1 boosts
// (lows pass more readily), 1 leaves the mid band equal to the authored scalar, >1 cuts (highs pass
// less) - a mass-law-like tilt (~6 dB/oct) so sound through a partition is low-pass "muffled".
// Bands 31.25 Hz .. 8 kHz. Because t is in [0,1], t=1 stays 1 and t=0 stays 0 in every band.
const float kTransmissionTilt[kNumBands] = {0.45f, 0.55f, 0.68f, 0.82f, 1.00f,
                                            1.25f, 1.60f, 2.10f, 2.70f};

// Distance model for propagation-graph edges. Within kPropRefDist the coupling is not distance-
// attenuated; beyond it, an inverse-distance geometric rolloff (refDist / distance) lowers the
// level, and per-band air absorption (dB/m, ~20 C / 50 % RH: negligible low, strong high) darkens
// the tail over distance. Air absorption is exponential in distance, so accumulating it per edge
// along a path equals absorbing over the total path length.
const float kPropRefDist = 4.0f;
const float kPropAirDbPerM[kNumBands] = {0.0000f, 0.0000f, 0.0003f, 0.0009f, 0.0018f,
                                         0.0033f, 0.0090f, 0.0280f, 0.0900f};

// Per-band distance attenuation in [0,1] for one edge of length `distance` metres (<=0 -> no
// attenuation). Geometric rolloff (broadband) * per-band air absorption (amplitude = 10^(-dB/20)).
void DistanceAttenuation(float distance, float outBands[kNumBands]) {
    if (distance <= kPropRefDist) {
        // Within the reference range only air absorption applies (tiny at short range).
        const float d = distance > 0.0f ? distance : 0.0f;
        for (int b = 0; b < kNumBands; ++b)
            outBands[b] = std::pow(10.0f, -kPropAirDbPerM[b] * d / 20.0f);
        return;
    }
    const float spread = kPropRefDist / distance; // inverse-distance beyond the reference range
    for (int b = 0; b < kNumBands; ++b) {
        const float air = std::pow(10.0f, -kPropAirDbPerM[b] * distance / 20.0f);
        float g = spread * air;
        outBands[b] = g < 0.0f ? 0.0f : (g > 1.0f ? 1.0f : g);
    }
}

float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Diffraction model tunables. A direct path with transmission >= kDiffractClearT needs no
// diffraction; a candidate opening must beat the direct path by kDiffractMargin and be at least
// kDiffractMinOpen open; the apparent source is pulled toward it by at least kDiffractMinPull
// (growing as the direct path is more blocked).
const float kDiffractClearT = 0.5f;
const float kDiffractMargin = 0.15f;
const float kDiffractMinOpen = 0.4f;
const float kDiffractMinPull = 0.3f;

const AcousticMaterial& MaterialOr(const AcousticMaterial* m) {
    return m != nullptr ? *m : DefaultMaterial();
}

} // namespace

int MaterialCount() { return kMaterialCount; }

const char* MaterialNameAt(int index) {
    return (index >= 0 && index < kMaterialCount) ? kMaterials[index].name : "";
}

const AcousticMaterial* MaterialAt(int index) {
    return (index >= 0 && index < kMaterialCount) ? &kMaterials[index].material : nullptr;
}

const AcousticMaterial* FindMaterial(const char* name) {
    if (name == nullptr) return nullptr;
    for (int i = 0; i < kMaterialCount; ++i)
        if (std::strcmp(name, kMaterials[i].name) == 0) return &kMaterials[i].material;
    return nullptr;
}

const AcousticMaterial& DefaultMaterial() { return kMaterials[0].material; }

float ReflectionCoefficient(const AcousticMaterial& m) {
    // Average absorption over the 500/1k/2k Hz bands (indices 4,5,6).
    const float avg = (m.absorption[4] + m.absorption[5] + m.absorption[6]) / 3.0f;
    const float r = 1.0f - avg;
    return std::sqrt(r > 0.0f ? r : 0.0f);
}

RoomAcoustics ComputeRoomAcoustics(const float dimensions[3],
                                   const AcousticMaterial* const walls[6], float reflectionGain,
                                   float reverbGain, float reverbTimeScale) {
    RoomAcoustics r;
    r.cutoffFrequencyHz = 800.0f;
    r.reflectionGain = reflectionGain > 0.0f ? reflectionGain : 0.0f;

    for (int i = 0; i < 6; ++i) r.reflectionCoefficients[i] = ReflectionCoefficient(MaterialOr(walls[i]));

    const float W = dimensions[0] > 0.1f ? dimensions[0] : 0.1f;
    const float H = dimensions[1] > 0.1f ? dimensions[1] : 0.1f;
    const float D = dimensions[2] > 0.1f ? dimensions[2] : 0.1f;
    const float V = W * H * D;
    // Per-face areas in reflection-coefficient order: [-x,+x]=H*D, [-y,+y]=W*D, [-z,+z]=W*H.
    const float area[6] = {H * D, H * D, W * D, W * D, W * H, W * H};
    float S = 0.0f;
    for (int i = 0; i < 6; ++i) S += area[i];
    if (S < 1e-3f) S = 1e-3f;
    const float timeScale = reverbTimeScale > 0.0f ? reverbTimeScale : 0.0f;

    for (int b = 0; b < kNumBands; ++b) {
        float A = 0.0f;
        for (int i = 0; i < 6; ++i) A += area[i] * MaterialOr(walls[i]).absorption[b];
        const float aBar = Clamp(A / S, 0.0f, 0.99f);          // clamp: avoid ln(0)
        const float denom = -S * std::log(1.0f - aBar) + 4.0f * kAir[b] * V; // Eyring + air
        float rt = denom > 1e-4f ? (0.161f * V / denom) : 0.0f;
        rt *= timeScale;
        r.rt60[b] = Clamp(rt, 0.0f, 12.0f);
    }
    // Late-reverb gain: the renderer's default (0.045) scaled by the caller's reverb-gain.
    r.reverbGain = 0.045f * (reverbGain > 0.0f ? reverbGain : 0.0f);
    return r;
}

float CombineTransmission(const float* transmissions, int count) {
    if (transmissions == nullptr || count <= 0) return 1.0f;
    float t = 1.0f;
    for (int i = 0; i < count; ++i) {
        t *= Clamp(transmissions[i], 0.0f, 1.0f);
        if (t < 1e-3f) return 0.0f; // fully blocked - stop early
    }
    return Clamp(t, 0.0f, 1.0f);
}

float PortalTransmission(const AcousticMaterial& closedMaterial, float openness) {
    const float o = Clamp(openness, 0.0f, 1.0f);
    return closedMaterial.transmission + (1.0f - closedMaterial.transmission) * o;
}

void MaterialTransmissionBands(const AcousticMaterial& m, float outBands[kNumBands]) {
    if (outBands == nullptr) return;
    const float t = Clamp(m.transmission, 0.0f, 1.0f);
    for (int b = 0; b < kNumBands; ++b)
        outBands[b] = Clamp(std::pow(t, kTransmissionTilt[b]), 0.0f, 1.0f);
}

void PortalTransmissionBands(const AcousticMaterial& closedMaterial, float openness,
                             float outBands[kNumBands]) {
    if (outBands == nullptr) return;
    const float o = Clamp(openness, 0.0f, 1.0f);
    float closed[kNumBands];
    MaterialTransmissionBands(closedMaterial, closed);
    for (int b = 0; b < kNumBands; ++b)
        outBands[b] = Clamp(closed[b] + (1.0f - closed[b]) * o, 0.0f, 1.0f);
}

void CombineTransmissionBands(const float* surfaceBands, int count, float outBands[kNumBands]) {
    if (outBands == nullptr) return;
    for (int b = 0; b < kNumBands; ++b) outBands[b] = 1.0f;
    if (surfaceBands == nullptr || count <= 0) return;
    for (int s = 0; s < count; ++s)
        for (int b = 0; b < kNumBands; ++b)
            outBands[b] = Clamp(outBands[b] * Clamp(surfaceBands[s * kNumBands + b], 0.0f, 1.0f),
                                0.0f, 1.0f);
}

void SolvePropagation(int roomCount, const PropagationEdge* edges, int edgeCount, int listenerRoom,
                      float* couplingOut) {
    if (couplingOut == nullptr || roomCount <= 0 || roomCount > kMaxPropagationRooms) return;
    for (int r = 0; r < roomCount * kNumBands; ++r) couplingOut[r] = 0.0f;
    if (listenerRoom < 0 || listenerRoom >= roomCount) return;
    for (int b = 0; b < kNumBands; ++b) couplingOut[listenerRoom * kNumBands + b] = 1.0f;
    if (edges == nullptr || edgeCount <= 0) return;
    if (edgeCount > kMaxPropagationEdges) edgeCount = kMaxPropagationEdges; // respect the bound
    // Precompute each edge's per-band effective gain = transmission * distance attenuation, once
    // (not per relaxation pass). Fixed stack buffer -> still allocation-free.
    float edgeGain[kMaxPropagationEdges * kNumBands];
    for (int e = 0; e < edgeCount; ++e) {
        float da[kNumBands];
        DistanceAttenuation(edges[e].distance, da);
        for (int b = 0; b < kNumBands; ++b)
            edgeGain[e * kNumBands + b] = Clamp(edges[e].transmission[b], 0.0f, 1.0f) * da[b];
    }
    // Max-product relaxation (Bellman-Ford). All per-band edge gains are in [0,1], so a path's
    // product only shrinks and each room's coupling only grows toward a bounded fixed point - no
    // negative cycles, converges in <= roomCount passes. Undirected: relax both directions, using
    // the just-improved value (Gauss-Seidel) to reach the fixed point sooner.
    for (int pass = 0; pass < roomCount; ++pass) {
        bool changed = false;
        for (int e = 0; e < edgeCount; ++e) {
            const PropagationEdge& ed = edges[e];
            if (ed.roomA < 0 || ed.roomA >= roomCount || ed.roomB < 0 || ed.roomB >= roomCount)
                continue;
            float* ca = &couplingOut[ed.roomA * kNumBands];
            float* cb = &couplingOut[ed.roomB * kNumBands];
            const float* g = &edgeGain[e * kNumBands];
            for (int b = 0; b < kNumBands; ++b) {
                const float na = cb[b] * g[b];
                if (na > ca[b] + 1e-6f) { ca[b] = na; changed = true; }
                const float nb = ca[b] * g[b];
                if (nb > cb[b] + 1e-6f) { cb[b] = nb; changed = true; }
            }
        }
        if (!changed) break;
    }
}

void DiffractedSourcePosition(const float src[3], const float listener[3], float directTransmission,
                              const DiffractionPortal* portals, int portalCount, float outPos[3]) {
    if (outPos == nullptr || src == nullptr) return;
    for (int b = 0; b < 3; ++b) outPos[b] = src[b];
    const float directT = Clamp(directTransmission, 0.0f, 1.0f);
    if (directT >= kDiffractClearT) return; // direct path mostly clear -> no diffraction
    if (portals == nullptr || portalCount <= 0) return;
    (void)listener; // the listener leg is already folded into portalToListener by the host
    float bestScore = directT + kDiffractMargin; // require a meaningfully clearer opening
    const float* best = nullptr;
    for (int i = 0; i < portalCount; ++i) {
        const DiffractionPortal& p = portals[i];
        if (p.openness < kDiffractMinOpen) continue;
        const float st = Clamp(p.srcToPortal, 0.0f, 1.0f);
        const float pl = Clamp(p.portalToListener, 0.0f, 1.0f);
        const float t = (st < pl ? st : pl) * Clamp(p.openness, 0.0f, 1.0f);
        if (t > bestScore) { bestScore = t; best = p.position; }
    }
    if (best != nullptr) {
        const float k = Clamp(1.0f - directT, kDiffractMinPull, 1.0f);
        for (int b = 0; b < 3; ++b) outPos[b] = src[b] + (best[b] - src[b]) * k;
    }
}

bool SelfTest() {
    bool ok = true;
    const char* fail = nullptr;
    #define HDSR_CHECK(c, msg) do { if (!(c)) { fail = (msg); ok = false; std::fprintf(stderr, "  [hdsr] FAIL: %s\n", (msg)); } } while (0)

    // Library integrity.
    HDSR_CHECK(MaterialCount() > 0, "empty material library");
    HDSR_CHECK(std::strcmp(MaterialNameAt(0), "Default") == 0, "material[0] != Default");
    for (int i = 0; i < MaterialCount(); ++i) {
        const AcousticMaterial* m = MaterialAt(i);
        HDSR_CHECK(m != nullptr, "MaterialAt returned null in range");
        if (!m) continue;
        for (int b = 0; b < kNumBands; ++b)
            HDSR_CHECK(m->absorption[b] >= 0.0f && m->absorption[b] <= 1.0f, "absorption out of [0,1]");
        HDSR_CHECK(m->scattering >= 0.0f && m->scattering <= 1.0f, "scattering out of [0,1]");
        HDSR_CHECK(m->transmission >= 0.0f && m->transmission <= 1.0f, "transmission out of [0,1]");
        HDSR_CHECK(FindMaterial(MaterialNameAt(i)) != nullptr, "FindMaterial missed a listed name");
    }
    HDSR_CHECK(FindMaterial("does-not-exist") == nullptr, "FindMaterial found a bogus name");

    const AcousticMaterial* marble = FindMaterial("Marble / Polished Stone");
    const AcousticMaterial* open = FindMaterial("Open / Air");
    const AcousticMaterial* fiber = FindMaterial("Fiberglass Insulation");
    HDSR_CHECK(marble && open && fiber, "required presets missing");
    if (marble && open && fiber) {
        // Reflection coefficients.
        HDSR_CHECK(ReflectionCoefficient(*marble) > 0.9f, "marble should reflect strongly");
        HDSR_CHECK(ReflectionCoefficient(*open) < 0.05f, "open/air should not reflect");

        // Room acoustics: hard room rings, open/absorptive is dry; big room's HF < LF (air).
        const float dims[3] = {8.0f, 4.0f, 6.0f};
        const AcousticMaterial* hardWalls[6] = {marble, marble, marble, marble, marble, marble};
        const AcousticMaterial* openWalls[6] = {open, open, open, open, open, open};
        const AcousticMaterial* softWalls[6] = {fiber, fiber, fiber, fiber, fiber, fiber};
        const RoomAcoustics hard = ComputeRoomAcoustics(dims, hardWalls, 1.0f, 1.0f, 1.0f);
        const RoomAcoustics dry = ComputeRoomAcoustics(dims, openWalls, 1.0f, 1.0f, 1.0f);
        const RoomAcoustics soft = ComputeRoomAcoustics(dims, softWalls, 1.0f, 1.0f, 1.0f);
        HDSR_CHECK(hard.rt60[4] > 2.0f, "hard room RT60 too short");
        HDSR_CHECK(dry.rt60[4] < 0.2f, "open room RT60 too long");
        HDSR_CHECK(soft.rt60[4] < 0.6f, "absorptive room RT60 too long");
        HDSR_CHECK(hard.rt60[4] > soft.rt60[4], "hard room should ring longer than absorptive");
        const float big[3] = {30.0f, 15.0f, 30.0f};
        const RoomAcoustics hall = ComputeRoomAcoustics(big, hardWalls, 1.0f, 1.0f, 1.0f);
        HDSR_CHECK(hall.rt60[8] < hall.rt60[0] - 1e-3f, "air absorption should shorten HF tail");
        // Null walls fall back to Default (no crash, valid coefficients).
        const AcousticMaterial* noWalls[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
        const RoomAcoustics def = ComputeRoomAcoustics(dims, noWalls, 1.0f, 1.0f, 1.0f);
        HDSR_CHECK(def.reflectionCoefficients[0] >= 0.0f && def.reflectionCoefficients[0] <= 1.0f,
                   "null-wall reflection coefficient out of range");

        // Propagation: product of transmissions; empty = clear; a blocker zeroes it.
        HDSR_CHECK(CombineTransmission(nullptr, 0) == 1.0f, "empty path should be clear");
        const float two[2] = {0.5f, 0.4f};
        HDSR_CHECK(std::fabs(CombineTransmission(two, 2) - 0.2f) < 1e-4f, "transmission product wrong");
        const float blocked[2] = {0.0f, 1.0f};
        HDSR_CHECK(CombineTransmission(blocked, 2) == 0.0f, "a blocker should zero transmission");
    }

    // Portal transmission.
    const AcousticMaterial* wood = FindMaterial("Wood Panel");
    HDSR_CHECK(wood != nullptr, "Wood Panel missing");
    if (wood) {
        HDSR_CHECK(std::fabs(PortalTransmission(*wood, 0.0f) - wood->transmission) < 1e-4f,
                   "closed portal != material transmission");
        HDSR_CHECK(std::fabs(PortalTransmission(*wood, 1.0f) - 1.0f) < 1e-4f, "open portal != 1");
        const float half = PortalTransmission(*wood, 0.5f);
        HDSR_CHECK(half > wood->transmission && half < 1.0f, "half-open portal not between");
    }

    // Frequency-dependent transmission: a partial blocker passes lows more than highs, with the mid
    // band anchored at the broadband scalar; a full opening stays flat; a portal opening brightens.
    const AcousticMaterial* drywall = FindMaterial("Drywall / Sheetrock");
    HDSR_CHECK(drywall != nullptr, "Drywall preset missing");
    if (drywall && open && wood) {
        float mb[kNumBands];
        MaterialTransmissionBands(*drywall, mb);
        HDSR_CHECK(std::fabs(mb[4] - drywall->transmission) < 1e-3f, "mid band != broadband scalar");
        HDSR_CHECK(mb[0] > mb[8] + 1e-3f, "lows should pass more than highs through a partition");
        HDSR_CHECK(mb[0] > drywall->transmission, "low band should exceed the broadband scalar");
        for (int b = 0; b < kNumBands; ++b)
            HDSR_CHECK(mb[b] >= 0.0f && mb[b] <= 1.0f, "material transmission band out of [0,1]");
        float ob[kNumBands];
        MaterialTransmissionBands(*open, ob);
        for (int b = 0; b < kNumBands; ++b)
            HDSR_CHECK(ob[b] > 0.999f, "an opening should transmit every band");
        // Portal bands: closed == material curve, fully open == flat 1, half-open brighter than shut.
        float closed[kNumBands], opened[kNumBands], halfB[kNumBands], wb[kNumBands];
        MaterialTransmissionBands(*wood, wb);
        PortalTransmissionBands(*wood, 0.0f, closed);
        PortalTransmissionBands(*wood, 1.0f, opened);
        PortalTransmissionBands(*wood, 0.5f, halfB);
        for (int b = 0; b < kNumBands; ++b) {
            HDSR_CHECK(std::fabs(closed[b] - wb[b]) < 1e-4f, "closed portal band != material band");
            HDSR_CHECK(opened[b] > 0.999f, "open portal band should pass fully");
            HDSR_CHECK(halfB[b] > closed[b] - 1e-4f && halfB[b] < opened[b] + 1e-4f,
                       "half-open portal band not between closed and open");
        }
        HDSR_CHECK(halfB[8] - closed[8] > halfB[0] - closed[0] - 1e-4f,
                   "opening a portal should brighten (raise highs) more than lows");
        // Per-band combine: product per band; two drywall panels darken more than one.
        float two[2 * kNumBands];
        for (int b = 0; b < kNumBands; ++b) { two[b] = mb[b]; two[kNumBands + b] = mb[b]; }
        float combined[kNumBands];
        CombineTransmissionBands(two, 2, combined);
        for (int b = 0; b < kNumBands; ++b)
            HDSR_CHECK(std::fabs(combined[b] - mb[b] * mb[b]) < 1e-4f, "per-band combine != product");
        float clear[kNumBands];
        CombineTransmissionBands(nullptr, 0, clear);
        for (int b = 0; b < kNumBands; ++b) HDSR_CHECK(clear[b] > 0.999f, "empty path should be clear");
    }

    // Room-to-room propagation graph: a chain L(0) -0.5- R1(1) -0.5- R2(2), listener in room 0.
    // R1 couples 0.5 (one hop), R2 couples 0.25 (two hops), and a weak direct 0-2 link (0.1) does
    // NOT win over the stronger two-hop path. Per-band tilt in an edge darkens the distant room.
    {
        PropagationEdge edges[3];
        edges[0].roomA = 0; edges[0].roomB = 1;
        edges[1].roomA = 1; edges[1].roomB = 2;
        edges[2].roomA = 0; edges[2].roomB = 2; // weak direct shortcut
        for (int b = 0; b < kNumBands; ++b) {
            edges[0].transmission[b] = 0.5f;
            edges[1].transmission[b] = 0.5f;
            edges[2].transmission[b] = 0.1f;
        }
        float coup[3 * kNumBands];
        SolvePropagation(3, edges, 3, 0, coup);
        HDSR_CHECK(std::fabs(coup[0 * kNumBands + 4] - 1.0f) < 1e-4f, "listener room coupling != 1");
        HDSR_CHECK(std::fabs(coup[1 * kNumBands + 4] - 0.5f) < 1e-3f, "one-hop room coupling != 0.5");
        HDSR_CHECK(std::fabs(coup[2 * kNumBands + 4] - 0.25f) < 1e-3f,
                   "two-hop path (0.25) should beat the weak direct shortcut (0.1)");
        // Unreachable room stays at 0.
        PropagationEdge one[1];
        one[0].roomA = 0; one[0].roomB = 1;
        for (int b = 0; b < kNumBands; ++b) one[0].transmission[b] = 0.5f;
        float c2[3 * kNumBands];
        SolvePropagation(3, one, 1, 0, c2);
        HDSR_CHECK(c2[2 * kNumBands + 4] == 0.0f, "unreachable room should have zero coupling");
        // Frequency-dependent edge: highs blocked more -> distant room's high band < low band.
        PropagationEdge fe[1];
        fe[0].roomA = 0; fe[0].roomB = 1;
        for (int b = 0; b < kNumBands; ++b) fe[0].transmission[b] = 0.8f - 0.07f * static_cast<float>(b);
        float c3[2 * kNumBands];
        SolvePropagation(2, fe, 1, 0, c3);
        HDSR_CHECK(c3[1 * kNumBands + 0] > c3[1 * kNumBands + 8] + 1e-3f,
                   "a darker-in-highs portal should couple lows more than highs");

        // Distance attenuation: with an OPEN edge (transmission 1), a far room couples less than a
        // near one, and a large distance darkens the highs (air absorption over distance).
        PropagationEdge de[1];
        de[0].roomA = 0; de[0].roomB = 1;
        for (int b = 0; b < kNumBands; ++b) de[0].transmission[b] = 1.0f;
        de[0].distance = 2.0f; // within the reference range -> ~no geometric rolloff
        float couNear[2 * kNumBands];
        SolvePropagation(2, de, 1, 0, couNear);
        de[0].distance = 40.0f; // far
        float couFar[2 * kNumBands];
        SolvePropagation(2, de, 1, 0, couFar);
        HDSR_CHECK(couNear[1 * kNumBands + 4] > 0.9f, "a near open room should couple almost fully");
        HDSR_CHECK(couFar[1 * kNumBands + 4] < couNear[1 * kNumBands + 4] - 0.1f,
                   "a far room should couple less than a near one (geometric rolloff)");
        HDSR_CHECK(couFar[1 * kNumBands + 4] > 0.0f, "a far room should still couple a little");
        HDSR_CHECK(couFar[1 * kNumBands + 0] > couFar[1 * kNumBands + 8] + 1e-3f,
                   "distance should darken the highs (air absorption over distance)");
    }

    // Diffraction: a clear direct path leaves the source where it is; an occluded direct path with a
    // clearer open portal to the side pulls the apparent source toward the portal; no clearer portal
    // (or no portals) leaves it unchanged.
    {
        const float src[3] = {0.0f, 0.0f, 0.0f};
        const float lis[3] = {10.0f, 0.0f, 0.0f};
        DiffractionPortal port;
        port.position[0] = 5.0f; port.position[1] = 5.0f; port.position[2] = 0.0f; // offset sideways
        port.openness = 1.0f;
        port.srcToPortal = 0.9f;
        port.portalToListener = 0.9f;
        float out[3];
        DiffractedSourcePosition(src, lis, 0.8f, &port, 1, out); // clear direct path
        HDSR_CHECK(out[0] == 0.0f && out[1] == 0.0f && out[2] == 0.0f,
                   "a clear direct path should not diffract");
        DiffractedSourcePosition(src, lis, 0.05f, &port, 1, out); // occluded, clearer portal
        HDSR_CHECK(out[1] > 0.5f, "an occluded source should be pulled toward the open portal");
        DiffractedSourcePosition(src, lis, 0.05f, nullptr, 0, out); // no portals
        HDSR_CHECK(out[0] == 0.0f && out[1] == 0.0f && out[2] == 0.0f,
                   "no portal -> apparent position equals the source");
        DiffractionPortal weak = port; // a portal no clearer than the direct path
        weak.srcToPortal = 0.05f; weak.portalToListener = 0.05f;
        DiffractedSourcePosition(src, lis, 0.05f, &weak, 1, out);
        HDSR_CHECK(out[1] == 0.0f, "a portal no clearer than the direct path should not pull");
        DiffractionPortal shut = port; // an opening below the openness threshold
        shut.openness = 0.2f;
        DiffractedSourcePosition(src, lis, 0.05f, &shut, 1, out);
        HDSR_CHECK(out[1] == 0.0f, "a barely-open portal should not diffract");
    }
    #undef HDSR_CHECK
    (void)fail;
    return ok;
}

} // namespace hdsr
