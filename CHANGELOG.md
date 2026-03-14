# Changelog

## [Unreleased]

### Added
- Haply Inverse3 + VerseGrip hand tracking plugin (`IsaacSimHaplyHandTracking`)
  - Minimal WebSocket client connecting to the Haply SDK service
  - Maps Inverse3 cursor position and VerseGrip orientation to Isaac Sim Hand Tracking C API
  - Configurable via `HAPLY_WEBSOCKET_HOST` and `HAPLY_WEBSOCKET_PORT` environment variables
  - Automatic reconnection with exponential backoff
  - No build-time SDK dependency (Haply SDK is a runtime-only dependency)
- `HaplyHandTrackerPrinter` CLI tool for debugging Haply hand tracker output
- CMake presets for Haply plugin: `linux-haply-default`, `linux-haply-debug`, `linux-haply-release`
- Combined `linux-all-default` preset for building both Manus and Haply plugins
