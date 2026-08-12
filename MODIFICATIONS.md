# Modifications from upstream

This fork tracks resonance-audio/resonance-audio. The goal is to keep the library
building on current toolchains while leaving the source and the public API unchanged.

## Summary

- No changes to the C++ source, the DSP, or the public API.
- No changes to how third party dependencies are handled. They are still fetched by
  the scripts in `third_party/` (`clone_core_deps.sh`), at the versions upstream pins.
- The full upstream tree is kept, including the Unity, FMOD, Wwise, and VST platform
  integrations and the tests.
- The standalone static library builds on Windows with Visual Studio 2022 and CMake 4.x.

## Build notes

Upstream declares `cmake_minimum_required(VERSION 3.4.1)`. CMake 4.x rejects a minimum
below 3.5. Rather than edit the CMake files, pass the policy floor at configure time:

    cmake -S . -B build -DBUILD_RESONANCE_AUDIO_API=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    cmake --build build --config Release

With that flag the `ResonanceAudioStatic` target configures, compiles, and links cleanly
on Visual Studio 2022 (x64, static `/MT` runtime). No source edits were required.

On Windows, cloning the repository needs long path support so the deep
`platforms/unity` asset paths check out:

    git clone -c core.longpaths=true <url>

## Files added by the fork

- `README.md` gained a short section describing the fork. The upstream README is kept
  below it.
- `MODIFICATIONS.md` (this file).
