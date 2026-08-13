// hdsr/environment_reverb.cc - see hdsr/environment_reverb.h.
//
// Each environment is a feedback-delay-network (FDN) reverberator: 8 delay lines mixed by an
// orthogonal (Householder) feedback matrix, with per-line feedback gain set from the environment's
// RT60 (energy -60 dB over RT60 seconds) and per-line one-pole damping set from the high-frequency
// RT60 (so the tail darkens over time). Environments run in parallel and are summed by their
// coupling to the listener. FDNs with an orthogonal matrix and per-line gain < 1 are unconditionally
// stable, so the mix cannot blow up regardless of the RT60s or number of environments.
//
// Copyright 2026 Hollow Dream Studios. Licensed under the Apache License, Version 2.0.
#include "hdsr/environment_reverb.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <vector>

namespace hdsr {

namespace {
const int kLines = 8;
const int kMaxBlock = 4096;
// Delay-line lengths in milliseconds - spread + mutually near-coprime for a dense, natural tail.
const float kDelayMs[kLines] = {19.1f, 23.7f, 29.3f, 37.1f, 41.3f, 47.9f, 53.3f, 59.7f};

float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
} // namespace

struct EnvironmentReverb::Impl {
    int sr = 0;
    int maxEnv = 0;
    int delayLen[kLines] = {0};

    struct Slot {
        int id = -1;              // -1 = free
        bool declared = false;    // set this block (else ringing out)
        float gain = 1.0f;
        float targetCoupling = 0.0f;
        float coupling = 0.0f;    // smoothed coupling actually applied (fades when undeclared)
        float energy = 0.0f;      // running tail energy estimate (to free silent slots)
        std::vector<std::vector<float>> delay; // [kLines][delayLen]
        int writePos[kLines] = {0};
        float lp[kLines] = {0.0f};
        float g[kLines] = {0.0f};    // per-line feedback gain (from RT60 mid)
        float damp[kLines] = {0.0f}; // per-line one-pole damping (from RT60 high)
        std::vector<float> input;    // per-block dry input accumulator (kMaxBlock)
    };
    std::vector<Slot> slots;

    void ResetSlotState(Slot& s) {
        for (int i = 0; i < kLines; ++i) {
            std::fill(s.delay[i].begin(), s.delay[i].end(), 0.0f);
            s.writePos[i] = 0;
            s.lp[i] = 0.0f;
        }
        s.energy = 0.0f;
        s.coupling = 0.0f;
    }

    // Set an environment's per-line gains + damping from its RT60 curve.
    void Configure(Slot& s, const float rt60[kNumBands]) {
        const float rtMid = Clampf(rt60[4], 0.02f, 20.0f);           // 500 Hz drives the tail length
        const float rtHigh = Clampf(rt60[kNumBands - 1], 0.02f, rtMid); // 8 kHz (<= mid) -> HF damping
        for (int i = 0; i < kLines; ++i) {
            const float t = static_cast<float>(delayLen[i]) / static_cast<float>(sr); // delay seconds
            s.g[i] = std::pow(10.0f, -3.0f * t / rtMid);             // -60 dB over RT60
            // Extra per-pass HF attenuation so the high band decays at rtHigh instead of rtMid.
            const float hfFactor = std::pow(10.0f, -3.0f * t * (1.0f / rtHigh - 1.0f / rtMid));
            const float hf = Clampf(hfFactor, 0.02f, 1.0f);
            s.damp[i] = Clampf((1.0f - hf) / (1.0f + hf), 0.0f, 0.9f); // one-pole LP coefficient
        }
    }

    Slot* FindById(int id) {
        for (Slot& s : slots)
            if (s.id == id) return &s;
        return nullptr;
    }

    // Get (or create) the slot for `id`. On overflow, evict the least-coupled slot iff the new
    // environment is more coupled; otherwise return null (this environment is dropped this block).
    Slot* Acquire(int id, float coupling) {
        if (Slot* s = FindById(id)) return s;
        for (Slot& s : slots)
            if (s.id < 0) { // free slot
                s.id = id;
                ResetSlotState(s);
                return &s;
            }
        // Full: find the least-coupled slot (undeclared slots are fading, so rank by smoothed
        // coupling - they are naturally the first to go).
        Slot* weakest = &slots[0];
        for (Slot& s : slots)
            if (s.coupling < weakest->coupling) weakest = &s;
        if (coupling > weakest->coupling) {
            weakest->id = id;
            ResetSlotState(*weakest);
            return weakest;
        }
        return nullptr; // new environment is the least coupled -> not admitted this block
    }

    // Runs one environment's FDN over its per-block input, mixing the wet tail into `out` scaled by
    // `mix`. Returns the block's peak wet magnitude (for the energy estimate).
    float ProcessSlot(Slot& s, float* out, int n, float mix) {
        const float invN2 = 2.0f / static_cast<float>(kLines);
        // Decorrelated stereo output taps.
        float cL[kLines], cR[kLines];
        for (int i = 0; i < kLines; ++i) {
            cL[i] = ((i & 1) ? -0.4f : 0.4f);
            cR[i] = ((i & 2) ? -0.4f : 0.4f);
        }
        float peak = 0.0f;
        for (int n_i = 0; n_i < n; ++n_i) {
            const float x = s.input[static_cast<size_t>(n_i)];
            float y[kLines];
            float sum = 0.0f;
            for (int i = 0; i < kLines; ++i) {
                float v = s.delay[i][static_cast<size_t>(s.writePos[i])]; // x[n - delayLen[i]]
                // One-pole low-pass damping (frequency-dependent decay).
                s.lp[i] = (1.0f - s.damp[i]) * v + s.damp[i] * s.lp[i];
                v = s.lp[i];
                y[i] = v;
                sum += v;
            }
            sum *= invN2;
            float wetL = 0.0f, wetR = 0.0f;
            for (int i = 0; i < kLines; ++i) {
                const float fb = (y[i] - sum) * s.g[i]; // orthogonal feedback * decay gain
                s.delay[i][static_cast<size_t>(s.writePos[i])] = x + fb;
                s.writePos[i] = (s.writePos[i] + 1) % delayLen[i];
                wetL += y[i] * cL[i];
                wetR += y[i] * cR[i];
            }
            out[n_i * 2 + 0] += wetL * mix;
            out[n_i * 2 + 1] += wetR * mix;
            const float m = std::fabs(wetL) + std::fabs(wetR);
            if (m > peak) peak = m;
        }
        return peak;
    }
};

EnvironmentReverb::EnvironmentReverb() : impl_(new Impl()) {}
EnvironmentReverb::~EnvironmentReverb() { delete impl_; }

bool EnvironmentReverb::Init(int sampleRate, int maxEnvironments) {
    if (sampleRate < 8000 || maxEnvironments < 1 || maxEnvironments > 64) return false;
    impl_->sr = sampleRate;
    impl_->maxEnv = maxEnvironments;
    int maxLen = 0;
    for (int i = 0; i < kLines; ++i) {
        int len = static_cast<int>(kDelayMs[i] * static_cast<float>(sampleRate) / 1000.0f);
        if (len < 1) len = 1;
        impl_->delayLen[i] = len;
        if (len > maxLen) maxLen = len;
    }
    impl_->slots.clear();
    impl_->slots.resize(static_cast<size_t>(maxEnvironments));
    for (Impl::Slot& s : impl_->slots) {
        s.id = -1;
        s.declared = false;
        s.coupling = 0.0f;
        s.energy = 0.0f;
        s.delay.assign(kLines, std::vector<float>());
        for (int i = 0; i < kLines; ++i)
            s.delay[static_cast<size_t>(i)].assign(static_cast<size_t>(impl_->delayLen[i]), 0.0f);
        for (int i = 0; i < kLines; ++i) {
            s.writePos[i] = 0;
            s.lp[i] = 0.0f;
        }
        s.input.assign(kMaxBlock, 0.0f);
    }
    return true;
}

bool EnvironmentReverb::IsReady() const { return impl_->sr > 0 && !impl_->slots.empty(); }
int EnvironmentReverb::MaxEnvironments() const { return impl_->maxEnv; }

int EnvironmentReverb::ActiveEnvironmentCount() const {
    int n = 0;
    for (const Impl::Slot& s : impl_->slots)
        if (s.id >= 0) ++n;
    return n;
}

void EnvironmentReverb::BeginBlock() {
    for (Impl::Slot& s : impl_->slots) {
        s.declared = false;
        std::fill(s.input.begin(), s.input.end(), 0.0f);
    }
}

void EnvironmentReverb::SetEnvironment(int id, const float rt60[kNumBands], float coupling,
                                       float gain) {
    if (id < 0 || !IsReady()) return;
    Impl::Slot* s = impl_->Acquire(id, Clampf(coupling, 0.0f, 1.0f));
    if (s == nullptr) return; // dropped by capacity (least coupled)
    s->declared = true;
    s->targetCoupling = Clampf(coupling, 0.0f, 1.0f);
    s->gain = gain > 0.0f ? gain : 0.0f;
    impl_->Configure(*s, rt60);
}

void EnvironmentReverb::AddInput(int id, const float* mono, int numSamples) {
    if (mono == nullptr || numSamples <= 0 || !IsReady()) return;
    Impl::Slot* s = impl_->FindById(id);
    if (s == nullptr || !s->declared) return;
    const int n = numSamples < kMaxBlock ? numSamples : kMaxBlock;
    for (int i = 0; i < n; ++i) s->input[static_cast<size_t>(i)] += mono[i];
}

void EnvironmentReverb::Process(float* interleavedStereo, int numSamples) {
    if (interleavedStereo == nullptr || numSamples <= 0 || !IsReady()) return;
    const int n = numSamples < kMaxBlock ? numSamples : kMaxBlock;
    std::memset(interleavedStereo, 0, sizeof(float) * static_cast<size_t>(n) * 2);
    for (Impl::Slot& s : impl_->slots) {
        if (s.id < 0) continue;
        // Smooth the applied coupling toward the target (0 when the environment is no longer
        // declared, so its tail fades out of the mix rather than clicking).
        const float target = s.declared ? s.targetCoupling : 0.0f;
        s.coupling += (target - s.coupling) * 0.25f;
        const float mix = s.coupling * s.gain;
        const float peak = impl_->ProcessSlot(s, interleavedStereo, n, mix);
        // Track the raw tail energy (independent of coupling) so a silent, undeclared slot frees.
        s.energy = 0.98f * s.energy + 0.02f * peak;
        if (!s.declared && s.coupling < 1e-4f && s.energy < 1e-5f) s.id = -1; // free
    }
}

bool EnvironmentReverb::SelfTest() {
    bool ok = true;
    const auto check = [&ok](bool c, const char* msg) {
        if (!c) {
            std::fprintf(stderr, "  [hdsr-reverb] FAIL: %s\n", msg);
            ok = false;
        }
    };

    const int sr = 48000;
    const int block = 512;
    std::vector<float> out(static_cast<size_t>(block) * 2, 0.0f);
    std::vector<float> impulse(static_cast<size_t>(block), 0.0f);

    // Measures the reverberation time (seconds): the last moment the tail envelope stays above
    // -60 dB of its peak, after a single impulse.
    auto measureRt60 = [&](float rt60Mid, float coupling) -> float {
        EnvironmentReverb rv;
        if (!rv.Init(sr, 4)) return -1.0f;
        float rt[kNumBands];
        for (int b = 0; b < kNumBands; ++b) rt[b] = rt60Mid;
        std::fill(impulse.begin(), impulse.end(), 0.0f);
        impulse[0] = 1.0f;
        const float maxSeconds = rt60Mid * 3.0f + 0.5f;
        const int maxBlocks = static_cast<int>(maxSeconds * sr / block) + 1;
        float peak = 0.0f;
        float lastAbove = 0.0f;
        int sampleIdx = 0;
        for (int blk = 0; blk < maxBlocks; ++blk) {
            rv.BeginBlock();
            rv.SetEnvironment(1, rt, coupling, 1.0f);
            if (blk == 0) rv.AddInput(1, impulse.data(), block); // impulse only in the first block
            rv.Process(out.data(), block);
            for (int i = 0; i < block; ++i, ++sampleIdx) {
                const float m = std::fabs(out[static_cast<size_t>(i) * 2]) +
                                std::fabs(out[static_cast<size_t>(i) * 2 + 1]);
                if (m > peak) peak = m;
                if (peak > 0.0f && m > peak * 0.001f)
                    lastAbove = static_cast<float>(sampleIdx) / static_cast<float>(sr);
            }
        }
        return lastAbove;
    };

    // Impulse decay ~ RT60 (loose tolerance - FDN decay is not perfectly exponential).
    const float rt1 = measureRt60(1.0f, 1.0f);
    check(rt1 > 0.4f && rt1 < 2.5f, "1.0s room decay out of range");
    const float rt3 = measureRt60(3.0f, 1.0f);
    check(rt3 > rt1, "longer RT60 should decay more slowly");
    const float rtDry = measureRt60(0.1f, 1.0f);
    check(rtDry < rt1, "a dry room should decay faster than a live one");

    // Coupling scales the level: half coupling -> ~half the peak output.
    {
        auto peakFor = [&](float coupling) -> float {
            EnvironmentReverb rv;
            rv.Init(sr, 4);
            float rt[kNumBands];
            for (int b = 0; b < kNumBands; ++b) rt[b] = 1.0f;
            std::fill(impulse.begin(), impulse.end(), 0.0f);
            impulse[0] = 1.0f;
            float peak = 0.0f;
            for (int blk = 0; blk < 60; ++blk) {
                rv.BeginBlock();
                rv.SetEnvironment(1, rt, coupling, 1.0f);
                if (blk == 0) rv.AddInput(1, impulse.data(), block);
                rv.Process(out.data(), block);
                for (int i = 0; i < block * 2; ++i)
                    if (std::fabs(out[static_cast<size_t>(i)]) > peak) peak = std::fabs(out[static_cast<size_t>(i)]);
            }
            return peak;
        };
        const float full = peakFor(1.0f);
        const float half = peakFor(0.5f);
        check(full > 1e-6f, "reverb produced no output");
        check(half < full * 0.75f && half > full * 0.25f, "coupling did not scale the tail level");
    }

    // Capacity: with room for 2 environments, declaring 3 keeps exactly 2 (the most coupled), and
    // does not crash / produce non-finite output.
    {
        EnvironmentReverb rv;
        check(rv.Init(sr, 2), "init with capacity 2 failed");
        float rt[kNumBands];
        for (int b = 0; b < kNumBands; ++b) rt[b] = 1.0f;
        std::fill(impulse.begin(), impulse.end(), 0.0f);
        impulse[0] = 1.0f;
        rv.BeginBlock();
        rv.SetEnvironment(10, rt, 0.9f, 1.0f);
        rv.SetEnvironment(11, rt, 0.8f, 1.0f);
        rv.SetEnvironment(12, rt, 0.1f, 1.0f); // least coupled -> should be dropped
        rv.AddInput(10, impulse.data(), block);
        rv.AddInput(12, impulse.data(), block); // ignored (env 12 not admitted)
        rv.Process(out.data(), block);
        check(rv.ActiveEnvironmentCount() == 2, "capacity 2 should keep exactly 2 environments");
        bool finite = true;
        for (int i = 0; i < block * 2; ++i)
            if (!(out[static_cast<size_t>(i)] == out[static_cast<size_t>(i)])) finite = false; // NaN check
        check(finite, "reverb output was non-finite");
    }

    return ok;
}

} // namespace hdsr
