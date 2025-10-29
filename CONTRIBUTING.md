# Contributing

Thanks for your interest in contributing! This document outlines how to build, test, and submit changes.

## Development Setup
- Linux x86_64, CMake ≥ 3.20, GCC/Clang with C++20
- Optional: Manus SDK for building the Manus plugin

### Build (default preset)
```bash
cmake --preset linux-manus-default
cmake --build --preset manus-default -j
```

### Build options
- `BUILD_MANUS_PLUGIN` (ON/OFF): build the Manus example plugin
- `MANUS_SDK_ROOT` (path): Manus SDK root containing `include/` and `lib/` or `lib64/`
- `COPY_MANUS_RUNTIME_IN_BUILD` (OFF by default): copy Manus runtime into build `lib/` (convenience only)

### Run
```bash
./build-manus-default/bin/ManusHandTrackerPrinter
```
If the Manus SDK runtime is not in RPATH, set:
```bash
LD_LIBRARY_PATH="$PWD/vendor/manus/ManusSDK/lib" ./build-manus-default/bin/ManusHandTrackerPrinter
```

## Code Style
- C++20, prefer clarity and readability
- Use meaningful identifiers; avoid over-abbreviations
- Keep warnings clean on GCC/Clang
- Format with clang-format (see `tools/` scripts)

## CI
GitHub Actions builds the default preset and runs a smoke test (`ldd` and basic launch). Please keep CI green.

## Pull Requests
1. Create a feature branch
2. Ensure builds pass locally and in CI
3. Describe motivation, changes, and testing in the PR
4. Link related issues

## License
- Your contributions are under the repository’s license unless stated otherwise
- Manus SDK is proprietary and not redistributed here; do not commit proprietary binaries

### Signing Your Work

* We require that all contributors "sign-off" on their commits. This certifies that the contribution is your original work, or you have rights to submit it under the same license, or a compatible license.

  * Any contribution which contains commits that are not Signed-Off will not be accepted.

* To sign off on a commit you simply use the `--signoff` (or `-s`) option when committing your changes:
  ```bash
  $ git commit -s -m "Add cool feature."
  ```
  This will append the following to your commit message:
  ```
  Signed-off-by: Your Name <your@email.com>
  ```

* Full text of the DCO:

  ```
    Developer Certificate of Origin
    Version 1.1

    Copyright (C) 2004, 2006 The Linux Foundation and its contributors.
    1 Letterman Drive
    Suite D4700
    San Francisco, CA, 94129

    Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.
  ```

  ```
    Developer's Certificate of Origin 1.1

    By making a contribution to this project, I certify that:

    (a) The contribution was created in whole or in part by me and I have the right to submit it under the open source license indicated in the file; or

    (b) The contribution is based upon previous work that, to the best of my knowledge, is covered under an appropriate open source license and I have the right under that license to submit that work with modifications, whether created in whole or in part by me, under the same open source license (unless I am permitted to submit under a different license), as indicated in the file; or

    (c) The contribution was provided directly to me by some other person who certified (a), (b) or (c) and I have not modified it.

    (d) I understand and agree that this project and the contribution are public and that a record of the contribution (including all personal information I submit with it, including my sign-off) is maintained indefinitely and may be redistributed consistent with this project or the open source license(s) involved.
  ```
