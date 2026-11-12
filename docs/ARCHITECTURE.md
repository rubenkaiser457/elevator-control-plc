# Architecture

## Hexagonal architecture (ports and adapters)

The control logic is pure and knows nothing about Modbus or the HMI. Both are adapters behind ports, so the same logic runs against the real PLC, the in-memory simulation and the unit tests.

```
HMI / tests --(driving port)--> Application --(driven ports)--> PLC (Modbus) / HMI sink
                                     |
                                     v
                          Domain (pure control logic)
```

| Port | Direction | Adapters |
|---|---|---|
| `OperatorPanel` | HMI to control: cabin calls, hall calls, emergency stop and reset | `MqttCommandRouter`, `InteractiveConsole` |
| `PlcGateway` | Read all sensor inputs, write all actuator outputs once per cycle | `ModbusPlcGateway`, `SimulatedPlcGateway` |
| `HmiGateway` | Publish status snapshots and events | `MqttHmiGateway`, `LoggingHmiGateway` |

### Package overview (`control/`)

```
de.htwg.sysarch.elevator
  domain/model/          Value objects: Level, Direction, Speed, Drive, MotorCommand, PlcInputs, PlcOutputs, ...
  domain/control/        ElevatorController (state machine), RequestStore (SCAN predicates), ElevatorStatus, events
  application/           ElevatorControlService (one cycle), ControlLoop (100 ms scheduler), ports
  infrastructure/        Adapters: modbus/, simulation/, mqtt/, console/, hmi/, config/
de.htwg.sysarch.mqtt     Shared message contract: topics and JSON DTOs (a copy lives in each module)
```

## Control logic

### Collective control (SCAN)

| Requirement | Implementation |
|---|---|
| 4 levels, 2 speeds per direction, crawl, off | `Level`, `Speed`, `Drive`, `MotorCommand` |
| Hall calls with a desired direction, stop if it matches the travel direction | `HallCall`, `RequestStore.shouldStopAt` |
| Cabin calls select a target level | `RequestStore` |
| Idle elevator moves toward the oldest request | Request timestamps, idle handling in `ElevatorController` |
| Continue while requests lie ahead, reverse when only requests behind remain | `RequestStore.hasRequestBeyond`, `ElevatorController.decideDirection` |
| Minimum dwell time of 6 s plus door cycle | Dwell timer in `ElevatorController` |
| Emergency stop, resume after reset | Emergency handling, `OperatorPanel.emergencyStop/reset` |

### Sensors and deceleration chain

Each level has up to five position sensors (level height 3.5 m):

| Sensor | Offset | Purpose |
|---|---|---|
| approach lower (`al`) | -1.5 m | Going up: slow down from v2 to v1 |
| safety lower (`sl`) | -50 mm | Going up: switch to crawl |
| reached (`r`) | 0 mm, +/-5 mm | Door-safe stop position |
| safety upper (`su`) | +50 mm | Going down: switch to crawl |
| approach upper (`au`) | +1.5 m | Going down: slow down from v2 to v1 |

Speeds: v2 about 1 m/s, v1 about 0.1 m/s, crawl -5 to 5 cm/s.

### Lessons from testing against the real PLC

The logic was first developed against the simulation and then tuned in live sessions with the lab PLC:

- **Door requests refused while the cabin still creeps:** The velocity register only has a resolution of 1 cm/s, so a reading of 0 can still mean slow movement. The controller now waits for several cycles of confirmed standstill inside the reached window before requesting the door.
- **The PLC samples the door-safe condition on the rising edge of the open request:** A refused request was later accepted at the same position. The controller now re-issues the open request every 2 s instead of waiting for a 10 s timeout.
- **Safety timeouts:** If the door does not open within 10 s the cabin re-levels; if crawling does not reach the sensor within 30 s the elevator stops and raises an alarm instead of searching forever.

## Modbus I/O mapping

All addresses are centralised in `ModbusIoMap` (Codesys `%QX` to coils, `%IX` to discrete inputs, `%QW/%IW/%ID` to registers).

**Outputs (written every cycle)**

| Symbol | Codesys | Meaning |
|---|---|---|
| POreset | %QX0.0 | Reset simulation / clear motor error |
| POdv2, POdv1 | %QX1.0, %QX1.1 | Motor down v2 / v1 |
| POuv1, POuv2 | %QX1.2, %QX1.3 | Motor up v1 / v2 |
| POdclose, POdopen | %QX1.4, %QX1.5 | Close / open door |
| POv_crawlSelect | %QW1 | Crawl speed, -5 to 5 cm/s |

**Inputs (read every cycle)**

| Symbol | Codesys | Meaning |
|---|---|---|
| PIs_lL{al,sl,r,su,au} | per level | Position sensors |
| PIs_dopened, PIs_dclosed | %IX10.0, %IX10.1 | Door open / closed |
| PIm_ready, PIm_on, PIm_error | %IX10.2 to %IX10.4 | Motor state |
| PIcycles | %ID1 | PLC cycle counter |
| PIs_v | %IW6 | Cabin velocity in cm/s |

## MQTT interface

Interface variables are writable in one direction only: the HMI reads status and events and writes commands.

| Topic | Direction | Payload | Retained |
|---|---|---|---|
| `<base>/status` | Control to HMI | `StatusMessage` | Yes |
| `<base>/event` | Control to HMI | `EventMessage` | No |
| `<base>/cmd` | HMI to control | `CommandMessage` | No |

```jsonc
// <base>/status
{"level":2,"direction":"UP","phase":"MOVING","door":"CLOSED","velocity":100,
 "emergencyActive":false,"motorError":false,"cabinCalls":[4],"hallUpCalls":[2],"hallDownCalls":[]}
// <base>/event
{"type":"ARRIVAL","level":3,"detail":""}
// <base>/cmd
{"type":"cabin","level":3}
{"type":"hall","level":2,"direction":"UP"}
{"type":"emergency","action":"engage"}
{"type":"reset"}
```

Without `--mqtt`, or if the broker is unreachable, the control system falls back to console logging and keeps running. It never depends on the HMI being available.

Broker credentials are never stored in the repository. They are passed at runtime via `-Dmqtt.username` / `-Dmqtt.password` or environment variables.
