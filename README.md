# Moonlight Embedded

> **This is a fork, not upstream.** It adds a V4L2 M2M + DRM/KMS zero-copy video
> backend for SoCs that expose a stateful mem2mem decoder and a KMS plane that can
> import dma-bufs (tested on Amlogic G12A / S905L3A, mainline 5.15). Upstream does
> not take this class of backend — standard V4L2 + DRM rendering is considered
> [moonlight-qt](https://github.com/moonlight-stream/moonlight-qt) territory, see
> [#932](https://github.com/moonlight-stream/moonlight-embedded/pull/932) — so it
> stays out of tree and is not proposed for merge. The device configuration that
> goes with it lives in
> [Moonlight-on-S905L3A-Armbian](https://github.com/benny391/Moonlight-on-S905L3A-Armbian).

 [![Build](https://img.shields.io/github/actions/workflow/status/moonlight-stream/moonlight-embedded/build.yml?branch=master)](https://github.com/moonlight-stream/moonlight-embedded/actions/workflows/build.yml?query=branch%3Amaster) [Nightly Build Downloads](https://nightly.link/moonlight-stream/moonlight-embedded/workflows/build/master)

Moonlight Embedded is an open source client for [Sunshine](https://github.com/LizardByte/Sunshine) and NVIDIA GameStream for embedded Linux systems, like Raspberry Pi, CuBox-i and ODROID. Moonlight allows you to stream your full collection of games and applications from your PC to other devices to play them remotely.

Moonlight also has [PC](https://github.com/moonlight-stream/moonlight-qt), [Android](https://github.com/moonlight-stream/moonlight-android), and [iOS](https://github.com/moonlight-stream/moonlight-ios) clients.

## Documentation

More information about installing and running Moonlight Embedded is available on the [wiki](https://github.com/moonlight-stream/moonlight-embedded/wiki).

## Bugs

Please check the wiki and old bug reports before submitting a new bug report.

Bugs can be reported to the [issue tracker](https://github.com/moonlight-stream/moonlight-embedded/issues).

## See also

[Moonlight-common-c](https://github.com/moonlight-stream/moonlight-common-c) is the shared codebase between different Moonlight implementations

## Contribute

1. Fork us
2. Write code
3. Send Pull Requests
