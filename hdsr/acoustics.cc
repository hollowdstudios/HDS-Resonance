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

float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

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
    #undef HDSR_CHECK
    (void)fail;
    return ok;
}

} // namespace hdsr
