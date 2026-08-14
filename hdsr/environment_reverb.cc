// hdsr/environment_reverb.cc - see hdsr/environment_reverb.h.
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
const float kDelayMs[kLines] = {19.1f, 23.7f, 29.3f, 37.1f, 41.3f, 47.9f, 53.3f, 59.7f};
const float kMaxPreDelayMs = 200.0f;

float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void CouplingLevelAndLp(const float coupling[kNumBands], float& level, float& lpA) {
    float gLow = 0.0f;
    for (int b = 0; b < 4; ++b) gLow += coupling[b]; // 31.25 .. 250 Hz
    gLow *= 0.25f;
    float gHigh = 0.0f;
    for (int b = kNumBands - 3; b < kNumBands; ++b) gHigh += coupling[b]; // 2k .. 8k Hz
    gHigh /= 3.0f;
    level = Clampf(gLow, 0.0f, 1.0f);
    const float r = level > 1e-5f ? Clampf(gHigh / level, 0.02f, 1.0f) : 1.0f;
    lpA = Clampf((1.0f - r) / (1.0f + r), 0.0f, 0.96f);
}
} // namespace

struct EnvironmentReverb::Impl {
    int sr = 0;
    int maxEnv = 0;
    int delayLen[kLines] = {0};
    int preFrames = 0; // propagation pre-delay ring length in frames (max delay + 1)

    struct Slot {
        int id = -1;
        bool declared = false;
        float gain = 1.0f;
        float targetCoupling = 0.0f;
        float coupling = 0.0f;
        float targetOutA = 0.0f;  // target output low-pass coeff (from the coupling spectrum tilt)
        float outA = 0.0f;        // smoothed output low-pass coeff
        float outLpL = 0.0f, outLpR = 0.0f;
        float targetPan = 0.0f;
        float pan = 0.0f;
        std::vector<float> preBuf;
        int preWrite = 0;
        int preDelaySamples = 0;   // current pre-delay in samples [0, preFrames-1]
        float energy = 0.0f;      // running tail energy estimate 
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
        s.outA = 0.0f;
        s.outLpL = 0.0f;
        s.outLpR = 0.0f;
        s.pan = 0.0f;
        std::fill(s.preBuf.begin(), s.preBuf.end(), 0.0f);
        s.preWrite = 0;
        s.preDelaySamples = 0; // a freshly (re)admitted environment starts un-delayed
    }

    void Configure(Slot& s, const float rt60[kNumBands]) {
        const float rtMid = Clampf(rt60[4], 0.02f, 20.0f);
        const float rtHigh = Clampf(rt60[kNumBands - 1], 0.02f, rtMid);
        for (int i = 0; i < kLines; ++i) {
            const float t = static_cast<float>(delayLen[i]) / static_cast<float>(sr); // delay seconds
            s.g[i] = std::pow(10.0f, -3.0f * t / rtMid);             // -60 dB over RT60
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

    static float EffectiveCoupling(const Slot& s) {
        const float t = s.declared ? s.targetCoupling : 0.0f;
        return s.coupling > t ? s.coupling : t;
    }

    Slot* Acquire(int id, float coupling) {
        if (Slot* s = FindById(id)) return s;
        for (Slot& s : slots)
            if (s.id < 0) { // free slot
                s.id = id;
                ResetSlotState(s);
                return &s;
            }
        // Full: evict the genuinely least-coupled slot, ranked by effective coupling this block.
        Slot* weakest = &slots[0];
        float weakestKey = EffectiveCoupling(slots[0]);
        for (Slot& s : slots) {
            const float k = EffectiveCoupling(s);
            if (k < weakestKey) {
                weakest = &s;
                weakestKey = k;
            }
        }
        if (coupling > weakestKey) {
            weakest->id = id;
            ResetSlotState(*weakest);
            return weakest;
        }
        return nullptr; // new environment is the least coupled -> not admitted this block
    }

    float ProcessSlot(Slot& s, float* out, int n, float mix) {
        const float invN2 = 2.0f / static_cast<float>(kLines);
        const float panL = 1.0f - 0.5f * (s.pan > 0.0f ? s.pan : 0.0f);
        const float panR = 1.0f - 0.5f * (s.pan < 0.0f ? -s.pan : 0.0f);
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
            const float m = std::fabs(wetL) + std::fabs(wetR);
            if (m > peak) peak = m;
            s.outLpL = (1.0f - s.outA) * wetL + s.outA * s.outLpL;
            s.outLpR = (1.0f - s.outA) * wetR + s.outA * s.outLpR;
            float dL = s.outLpL, dR = s.outLpR;
            if (preFrames > 0) {
                s.preBuf[static_cast<size_t>(s.preWrite) * 2 + 0] = s.outLpL;
                s.preBuf[static_cast<size_t>(s.preWrite) * 2 + 1] = s.outLpR;
                int r = s.preWrite - s.preDelaySamples;
                if (r < 0) r += preFrames;
                dL = s.preBuf[static_cast<size_t>(r) * 2 + 0];
                dR = s.preBuf[static_cast<size_t>(r) * 2 + 1];
                s.preWrite = (s.preWrite + 1) % preFrames;
            }
            out[n_i * 2 + 0] += dL * mix * panL;
            out[n_i * 2 + 1] += dR * mix * panR;
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
    // Propagation pre-delay ring: max delay in samples + 1 (so read never collides with write).
    impl_->preFrames = static_cast<int>(kMaxPreDelayMs * static_cast<float>(sampleRate) / 1000.0f) + 1;
    impl_->slots.clear();
    impl_->slots.resize(static_cast<size_t>(maxEnvironments));
    for (Impl::Slot& s : impl_->slots) {
        s.id = -1;
        s.declared = false;
        s.coupling = 0.0f;
        s.outA = 0.0f;
        s.outLpL = 0.0f;
        s.outLpR = 0.0f;
        s.preBuf.assign(static_cast<size_t>(impl_->preFrames) * 2, 0.0f);
        s.preWrite = 0;
        s.preDelaySamples = 0;
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
    // The scalar case is a flat coupling curve (every band equal)
    const float c = Clampf(coupling, 0.0f, 1.0f);
    float bands[kNumBands];
    for (int b = 0; b < kNumBands; ++b) bands[b] = c;
    SetEnvironment(id, rt60, bands, gain);
}

void EnvironmentReverb::SetEnvironment(int id, const float rt60[kNumBands],
                                       const float coupling[kNumBands], float gain) {
    if (id < 0 || coupling == nullptr || !IsReady()) return;
    float level = 0.0f, lpA = 0.0f;
    CouplingLevelAndLp(coupling, level, lpA);
    Impl::Slot* s = impl_->Acquire(id, level); // capacity ranks by the broadband coupling level
    if (s == nullptr) return;                  // dropped by capacity (least coupled)
    s->declared = true;
    s->targetCoupling = level;
    s->targetOutA = lpA;
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

void EnvironmentReverb::SetEnvironmentPan(int id, float pan) {
    if (!IsReady()) return;
    Impl::Slot* s = impl_->FindById(id);
    if (s == nullptr) return;
    s->targetPan = Clampf(pan, -1.0f, 1.0f);
}

void EnvironmentReverb::SetEnvironmentPreDelay(int id, float seconds) {
    if (!IsReady()) return;
    Impl::Slot* s = impl_->FindById(id);
    if (s == nullptr) return;
    int samples = seconds > 0.0f ? static_cast<int>(seconds * static_cast<float>(impl_->sr)) : 0;
    if (samples < 0) samples = 0;
    if (samples > impl_->preFrames - 1) samples = impl_->preFrames - 1; // clamp to the ring
    s->preDelaySamples = samples;
}

void EnvironmentReverb::Process(float* interleavedStereo, int numSamples) {
    if (interleavedStereo == nullptr || numSamples <= 0 || !IsReady()) return;
    const int n = numSamples < kMaxBlock ? numSamples : kMaxBlock;
    std::memset(interleavedStereo, 0, sizeof(float) * static_cast<size_t>(n) * 2);
    for (Impl::Slot& s : impl_->slots) {
        if (s.id < 0) continue;
        const float target = s.declared ? s.targetCoupling : 0.0f;
        s.coupling += (target - s.coupling) * 0.25f;
        if (s.declared) s.outA += (s.targetOutA - s.outA) * 0.25f;
        if (s.declared) s.pan += (s.targetPan - s.pan) * 0.25f;
        const float mix = s.coupling * s.gain;
        const float peak = impl_->ProcessSlot(s, interleavedStereo, n, mix);
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
    
    {
        EnvironmentReverb rv;
        rv.Init(sr, 1);
        float rt[kNumBands];
        for (int b = 0; b < kNumBands; ++b) rt[b] = 1.0f;
        std::fill(impulse.begin(), impulse.end(), 0.0f);
        impulse[0] = 1.0f;
        float peakWeak = 0.0f;
        for (int blk = 0; blk < 30; ++blk) {
            rv.BeginBlock();
            rv.SetEnvironment(20, rt, 0.9f, 1.0f); // strong, declared first
            rv.SetEnvironment(21, rt, 0.1f, 1.0f); // weak, overflow -> must be rejected
            if (blk == 0) rv.AddInput(21, impulse.data(), block); // feed the room that must be rejected
            rv.Process(out.data(), block);
            for (int i = 0; i < block * 2; ++i)
                if (std::fabs(out[static_cast<size_t>(i)]) > peakWeak) peakWeak = std::fabs(out[static_cast<size_t>(i)]);
        }
        check(rv.ActiveEnvironmentCount() == 1, "capacity 1 should hold exactly 1 environment");
        check(peakWeak < 1e-6f, "the least-coupled room must be rejected (not evict the most-coupled)");
    }

    {
        auto brightness = [&](bool dark) -> float {
            EnvironmentReverb rv;
            rv.Init(sr, 4);
            float rt[kNumBands];
            for (int b = 0; b < kNumBands; ++b) rt[b] = 1.0f;
            float coup[kNumBands];
            for (int b = 0; b < kNumBands; ++b)
                coup[b] = dark ? (0.8f - 0.09f * static_cast<float>(b)) : 0.8f; // dark: highs fall off
            std::fill(impulse.begin(), impulse.end(), 0.0f);
            impulse[0] = 1.0f;
            double sumSq = 0.0, diffSq = 0.0;
            float prev = 0.0f;
            for (int blk = 0; blk < 60; ++blk) {
                rv.BeginBlock();
                rv.SetEnvironment(1, rt, coup, 1.0f);
                if (blk == 0) rv.AddInput(1, impulse.data(), block);
                rv.Process(out.data(), block);
                for (int i = 0; i < block; ++i) {
                    const float v = out[static_cast<size_t>(i) * 2]; // left channel
                    sumSq += static_cast<double>(v) * v;
                    const float d = v - prev;
                    diffSq += static_cast<double>(d) * d;
                    prev = v;
                }
            }
            return sumSq > 1e-12 ? static_cast<float>(diffSq / sumSq) : 0.0f;
        };
        const float flatB = brightness(false);
        const float darkB = brightness(true);
        check(flatB > 1e-6f, "per-band reverb produced no output");
        check(darkB < flatB * 0.9f, "a darker-in-highs coupling should reduce tail brightness");
    }

    {
        auto onsetSample = [&](float preSec) -> int {
            EnvironmentReverb rv;
            rv.Init(sr, 1);
            float rt[kNumBands];
            for (int b = 0; b < kNumBands; ++b) rt[b] = 1.0f;
            std::fill(impulse.begin(), impulse.end(), 0.0f);
            impulse[0] = 1.0f;
            int idx = -1, sampleIdx = 0;
            for (int blk = 0; blk < 40 && idx < 0; ++blk) {
                rv.BeginBlock();
                rv.SetEnvironment(1, rt, 1.0f, 1.0f);
                rv.SetEnvironmentPreDelay(1, preSec);
                if (blk == 0) rv.AddInput(1, impulse.data(), block);
                rv.Process(out.data(), block);
                for (int i = 0; i < block; ++i, ++sampleIdx) {
                    const float m = std::fabs(out[static_cast<size_t>(i) * 2]) +
                                    std::fabs(out[static_cast<size_t>(i) * 2 + 1]);
                    if (m > 1e-5f) { idx = sampleIdx; break; }
                }
            }
            return idx;
        };
        const int on0 = onsetSample(0.0f);
        const int on50 = onsetSample(0.05f);
        check(on0 >= 0 && on50 >= 0, "pre-delay onset not found");
        const int expected = static_cast<int>(0.05f * static_cast<float>(sr));
        check(std::abs((on50 - on0) - expected) < 200,
              "pre-delay should shift the tail onset by ~distance/speed-of-sound");
    }

    // Directional pan: a fully-right-panned environment biases the tail toward the right channel
    {
        auto lrEnergy = [&](float pan, double& sumL, double& sumR) {
            EnvironmentReverb rv;
            rv.Init(sr, 2);
            float rt[kNumBands];
            for (int b = 0; b < kNumBands; ++b) rt[b] = 1.0f;
            std::fill(impulse.begin(), impulse.end(), 0.0f);
            impulse[0] = 1.0f;
            sumL = 0.0;
            sumR = 0.0;
            for (int blk = 0; blk < 40; ++blk) {
                rv.BeginBlock();
                rv.SetEnvironment(1, rt, 1.0f, 1.0f);
                rv.SetEnvironmentPan(1, pan);
                if (blk == 0) rv.AddInput(1, impulse.data(), block);
                rv.Process(out.data(), block);
                for (int i = 0; i < block; ++i) {
                    sumL += std::fabs(out[static_cast<size_t>(i) * 2]);
                    sumR += std::fabs(out[static_cast<size_t>(i) * 2 + 1]);
                }
            }
        };
        double rL = 0.0, rR = 0.0, cL2 = 0.0, cR2 = 0.0;
        lrEnergy(1.0f, rL, rR);   // full right
        lrEnergy(0.0f, cL2, cR2); // centred
        check(rR > rL * 1.2, "a right-panned environment should bias the tail to the right");
        check(cL2 > 1e-9 && std::fabs(cL2 - cR2) < cL2 * 0.5,
              "a centred (pan 0) environment should stay roughly balanced");
    }

    return ok;
}

} // namespace hdsr
