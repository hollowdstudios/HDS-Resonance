# Modifications from upstream

This fork tracks resonance-audio/resonance-audio. It keeps the upstream renderer (`vraudio`) and its
public API unchanged, and adds a new engine-agnostic acoustics **model** layer (`hdsr`) beside it.

## Renderer (`vraudio`) — unchanged

- No changes to the upstream C++ source, the DSP, or the public API
  (`resonance_audio/api/resonance_audio_api.h`).
- No changes to how third party dependencies are handled. They are still fetched by the scripts in
  `third_party/` (`clone_core_deps.sh`), at the versions upstream pins.
- The full upstream tree is kept, including the Unity, FMOD, Wwise, and VST platform integrations and
  the tests.

## Acoustics model (`hdsr`) — added

A new, self-contained layer under `hdsr/`, compiled into the same static library. It depends only on
the C++ standard library — no engine, physics, or file-format types — so any consumer can adopt it.

- `hdsr/acoustics.{h,cc}` — frequency-dependent acoustic materials, shoebox room acoustics (Eyring
  RT60 + air absorption), per-band transmission through surfaces and portal openings, the room-to-room
  propagation graph (`SolvePropagation` — best-path per-band coupling with distance and air-absorption
  decay along the path), and the diffraction model (`DiffractedSourcePosition`). Includes a headless
  `SelfTest()`.
- `hdsr/environment_reverb.{h,cc}` — `EnvironmentReverb`, a multi-environment reverberator: several
  rooms reverberate at once, each an RT60-tuned feedback-delay network, mixed by per-band coupling and
  shaped by distance (a muffling low-pass), propagation pre-delay, and stereo pan. Includes a
  `SelfTest()`.
- `docs/AcousticsLibrary.md` — the design and API reference for the layer.

The only edit to the upstream build is that `resonance_audio/CMakeLists.txt` adds the two `hdsr/`
translation units to the `ResonanceAudioStatic` sources; the fork root is already on the include path,
so consumers include `"hdsr/acoustics.h"` and `"hdsr/environment_reverb.h"`.

## Build notes

Upstream declares `cmake_minimum_required(VERSION 3.4.1)`. CMake 4.x rejects a minimum below 3.5.
Rather than edit the CMake files, pass the policy floor at configure time:

    cmake -S . -B build -DBUILD_RESONANCE_AUDIO_API=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    cmake --build build --config Release

With that flag the `ResonanceAudioStatic` target configures, compiles, and links cleanly on Visual
Studio 2022 (x64, static `/MT` runtime).

On Windows, cloning the repository needs long path support so the deep `platforms/unity` asset paths
check out:

    git clone -c core.longpaths=true <url>
