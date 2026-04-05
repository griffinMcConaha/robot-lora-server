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

## Backend Connection Model

The backend should publish:

- `connectivity.overall`
- `connectivity.backend`
- `connectivity.baseStation`
- `connectivity.robot`
- `connectivity.commandPath`

These are the source of truth for customer-facing readiness.
