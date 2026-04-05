# System Flow Contract

This is the shared operational contract for:

- app
- backend server
- base station
- gateway
- robot

## Connection States

Use these exact values across the system:

- `offline`
- `connecting`
- `online`
- `degraded`
- `stale`
- `faulted`
- `ready`

## Robot States

Use these exact values across the system:

- `IDLE`
- `MANUAL`
- `AUTO`
- `PAUSE`
- `ERROR`
- `ESTOP`

## Command Lifecycle

Commands should move through these states as evidence is gained:

- `queued`
- `sent`
- `forwarded`
- `acknowledged`
- `applied`
- `failed`
- `timed_out`

## Command Transport Stages

For the customer-facing transport story, normalize recent commands into these exact stages:

- `backend_queued`
- `base_station_forwarded`
- `lora_acknowledged`
- `robot_applied`
- `failed`
- `timed_out`

These are derived from the underlying command lifecycle plus bridge/base-station evidence.

## Command Metadata

Whenever possible, preserve these fields:

- `commandId`
- `cmd`
- `source`
- `issuedAt`
- `status`

## Base Station Status Fields

Structured status should expose:

- `status_version`
- `state`
- `mode`
- `wifi_link_state`
- `lora_link_state`
- `last_cmd`
- `last_cmd_id`
- `last_cmd_status`
- `queue_depth`
- `ack_count`
- `last_ack`
- `last_lora`

## Gateway And Robot ACK Notes

Current safe interpretation across the downstream path is:

- Gateway ACKs confirm LoRa frame receipt/forwarding, not robot execution.
- Robot ACKs like `ACK:STATE:*`, `ACK:WPCLEAR`, `ACK:WP:<idx>`, and `ACK:WPLOAD:<n>` confirm robot-side receipt.
- Robot telemetry state confirmation is the evidence for `applied`.

Normalized ACK categories exposed by the backend should be:

- `gateway_frame`
- `robot_state`
- `waypoint_clear`
- `waypoint_add`
- `waypoint_load`

Because STM32 waypoint parsing is strict, inline command metadata should not be added to `WP:` or `WPLOAD:` payloads unless the robot firmware is updated first.

## Backend Connection Model

The backend should publish:

- `connectivity.overall`
- `connectivity.backend`
- `connectivity.baseStation`
- `connectivity.robot`
- `connectivity.commandPath`

These are the source of truth for customer-facing readiness.
