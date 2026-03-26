# Robot LoRa Server - Release Notes (2026-03-26)

## Summary
This release hardens production safety, reliability, and operations across the LoRa server stack.

## Included changes

### Safety and readiness (P0)
- Added telemetry stale fail-safe policy with configurable action (`ESTOP` or `PAUSE`).
- Added geofence breach fail-safe policy with configurable action.
- Added cooldowns and monitor interval controls for safety checks.
- Added richer health endpoint readiness checks (`db`, `bridge`, `telemetry`) with `503` on not-ready conditions.

### Reliability and observability (P1/P2)
- Added bridge transient retry controls and degraded-state reporting.
- Added app-scoped metrics endpoint: `/api/metrics`.
- Added API key rotation support via multi-key env vars:
  - `APP_API_KEYS`
  - `BOARD_API_KEYS`
- Added command + telemetry request rate limiting.
- Added event retention pruning controls.

### Data and API contract updates
- Added `safety.action` event type to contracts.
- Added metrics path to API contracts.
- Added DB ping and event pruning helpers.

### Test coverage
- Added/expanded HIL tests for:
  - telemetry stale fail-safe mission abort path
  - geofence fail-safe mission abort path
  - health degraded behavior while mission is running with stale telemetry

### DevOps / automation
- Added server CI workflow:
  - `.github/workflows/server-ci.yml`
- Added release gate script:
  - `npm run release:gate`
- Added key-rotation validation script:
  - `npm run ops:key-rotation:check`
- Added SQLite backup/restore drill script:
  - `npm run ops:sqlite:drill`

## Validation status
- `npm run test:hil`: pass
- `npm run release:gate`: pass (live endpoint checks optional)
- `npm run ops:sqlite:drill`: pass
- `npm run ops:key-rotation:check`: pass (overlap and revoked-key modes)

## Deployment notes
- Ensure app sends `EXPO_PUBLIC_APP_API_KEY`.
- For production auth rotation, deploy with overlap keys first, then retire old keys.
- Keep persistent SQLite volume mounted and writable.

## Still external to this repo
- App CI lane in `robot-app-main`.
- Firmware CI lanes in STM32 and ESP-IDF repos.
- Hardware soak and real-device ESP-IDF `/health`/degraded verification.
