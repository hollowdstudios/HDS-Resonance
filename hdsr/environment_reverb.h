// hdsr/environment_reverb.h - HDS-Resonance environment-aware multi-reverb (public API).
//
// A stereo reverberator that runs SEVERAL acoustic environments at once. Each active environment
// (a room) has its own reverberation time (RT60 per octave band) and contributes its own reverb
// TAIL to the listener, scaled by how acoustically COUPLED it is to the listener (portal /
// propagation coupling supplied by the host). The library MANAGES the set of environments - it
// mixes ALL active environments up to an explicit capacity (a performance limit, not a
// "loudest-room-wins" selection) - and owns the reverb DSP (a per-environment feedback-delay
// network tuned to the environment's RT60). Each environment persists its tail across blocks, so a
// source in the next room keeps ringing there after it stops and leaks to the listener through the
// coupling.
//
// The host engine supplies only: room topology (which environments are active + their RT60), the
// per-environment coupling to the listener, and the source audio routed into the environment each
// source occupies. The library needs no engine, physics, or geometry types. C++11; all buffers are
// pre-allocated in Init (no allocation in the per-block hot path).
//
// Copyright 2026 Hollow Dream Studios. Licensed under the Apache License, Version 2.0.
#ifndef HDSR_ENVIRONMENT_REVERB_H_
#define HDSR_ENVIRONMENT_REVERB_H_

#include "hdsr/acoustics.h" // kNumBands

namespace hdsr {

class EnvironmentReverb {
public:
    EnvironmentReverb();
    ~EnvironmentReverb();
    EnvironmentReverb(const EnvironmentReverb&) = delete;
    EnvironmentReverb& operator=(const EnvironmentReverb&) = delete;

    // Prepares the reverberator. `maxEnvironments` is the number of environments processed
    // simultaneously - an explicit performance capacity; every active environment up to this many
    // is mixed (overflow drops the LEAST-coupled environment, not the quietest source). Returns
    // false on bad arguments. Safe to call again to reconfigure (resets all tails).
    bool Init(int sampleRate, int maxEnvironments);
    bool IsReady() const;
    int MaxEnvironments() const;
    // Number of environments currently held (active this block or still ringing out their tail).
    // Never exceeds MaxEnvironments().
    int ActiveEnvironmentCount() const;

    // -- Per-block usage --------------------------------------------------------------------------
    //   BeginBlock();
    //   for each active environment e:  SetEnvironment(e.id, e.rt60, e.coupling, e.gain);
    //   for each source s:              AddInput(s.environmentId, s.mono, numSamples);
    //   Process(interleavedStereoOut, numSamples);
    // `numSamples` may vary block to block (up to the internal cap); environments/tails persist.

    // Marks the start of a processing block: every environment becomes "not declared this block"
    // (an environment left undeclared keeps ringing out its tail, then frees its slot when silent).
    void BeginBlock();

    // Declares environment `id` active for this block with reverberation time `rt60` (seconds, per
    // octave band), `coupling` in [0,1] (fraction of this environment's tail reaching the listener;
    // 1 = the listener is inside it), and a linear `gain`. An id already ringing keeps its tail
    // (continuous). If the capacity is exceeded, the least-coupled environment is dropped.
    void SetEnvironment(int id, const float rt60[kNumBands], float coupling, float gain);

    // Per-band coupling overload: `coupling[b]` in [0,1] is the fraction of this environment's tail
    // reaching the listener in octave band b (from the room-to-room propagation graph - see
    // hdsr::SolvePropagation). The tail is mixed at the low-band coupling level AND low-pass filtered
    // by the coupling spectrum's high-to-low ratio, so a distant room bleeds through DARKENED
    // (muffled), not merely quieter. The scalar overload above is the flat special case (all bands
    // equal -> no filtering). Same capacity/continuity rules.
    void SetEnvironment(int id, const float rt60[kNumBands], const float coupling[kNumBands],
                        float gain);

    // Sets environment `id`'s stereo PAN in [-1, +1] (-1 = full left, 0 = centred/diffuse, +1 =
    // full right): the direction its reverb arrives from, so a room reached through a doorway off to
    // one side bleeds in from THAT side instead of sounding centred/mono. The listener's own room
    // should stay 0 (its reverb surrounds them). It is a gentle bias (the tail keeps its stereo
    // width), smoothed as the listener turns. Call after SetEnvironment (no-op for an unadmitted id).
    void SetEnvironmentPan(int id, float pan);

    // Sets environment `id`'s propagation PRE-DELAY: the number of seconds its reverb tail is
    // delayed before it mixes into the listener's output, because the room's reverberant field takes
    // time to travel to the listener (distance / speed-of-sound). The listener's own room is ~0; a
    // distant room's tail arrives audibly later. Call after SetEnvironment (no-op for an id not
    // declared / admitted this block). Clamped to an internal maximum (~200 ms); 0 = no delay (the
    // output is then bit-identical to the no-pre-delay path). The tail is diffuse, so the integer
    // delay length may change block to block (listener moving) without audible artifacts.
    void SetEnvironmentPreDelay(int id, float seconds);

    // Adds a source's mono audio into environment `id`'s reverb input for this block (sources in
    // the same environment sum). Ignored for ids not declared this block or already evicted.
    void AddInput(int id, const float* mono, int numSamples);

    // Renders the block: processes every active/ringing environment's reverb, mixes each by its
    // coupling*gain, and writes interleaved stereo (numSamples*2). Then clears the per-block input.
    void Process(float* interleavedStereo, int numSamples);

    // Headless self-test (impulse-response decay ~ RT60, coupling scaling, capacity eviction,
    // longer RT60 -> slower decay). Returns true on pass.
    static bool SelfTest();

private:
    struct Impl;
    Impl* impl_;
};

} // namespace hdsr

#endif // HDSR_ENVIRONMENT_REVERB_H_
