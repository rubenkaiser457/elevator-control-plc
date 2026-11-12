# Elevator Control System with Modbus/TCP and MQTT

Control software for a 4-level elevator running as a simulation on an industrial PLC. The control system acts as a **Modbus/TCP master** with a 100 ms control cycle, implements the full **collective control (SCAN) logic** and exposes its state and commands to a separate **HMI over MQTT**.

Team project for the course *System Architecture* at HTWG Konstanz (summer 2026). The project was tested against the real PLC in the university's automation lab.

## Features

- **Collective control (SCAN):** keeps travelling in one direction while requests lie ahead, then reverses; serves hall calls in the travel direction
- **Two-speed approach with deceleration chain:** v2 while travelling, v1 at the approach sensor, crawl at the safety sensor, stop at the door-safe position
- **Door handling** with minimum dwell time, timeouts and automatic re-levelling
- **Emergency stop** with resume after reset, motor error handling
- **Robust against real-world PLC behaviour:** settle cycles before opening the door, retry edges for refused door requests, crawl timeout as a safety stop
- **In-memory PLC simulation** for development and unit tests without lab access
- **MQTT HMI interface** (JSON) with a graphical Swing HMI and a console HMI
- **Hexagonal architecture:** the control logic is pure Java with no dependency on Modbus or MQTT

## Tech stack

| Area | Technologies |
|---|---|
| Language / build | Java 11, Maven |
| Industrial communication | Modbus/TCP (Codesys PLC), easymodbus |
| Messaging | MQTT (Eclipse Paho, Mosquitto), JSON (Gson) |
| HMI | Java Swing |
| Testing | JUnit 5 |

## Architecture

The control system and the HMI are two separate processes. They share only the MQTT message contract, never code.

```
   HMI (Swing / console)            Control system                       PLC
 +----------------------+        +-------------------------------+     +-------------------+
 | buttons, shaft view  | <----- | Infrastructure (adapters)     | --> | elevator          |
 | live status, log     | status | Application (ports, 100 ms)   | Mod | simulation        |
 |                      | -----> | Domain (pure control logic)   | bus | (sensors, motor,  |
 +----------------------+  cmd   +-------------------------------+ TCP | door)             |
          MQTT                                                         +-------------------+
```

- **Domain:** the control state machine (`ElevatorController`) and request handling (`RequestStore`), free of any I/O
- **Application:** one control cycle every 100 ms; ports `OperatorPanel` (commands in), `PlcGateway` (sensors/actuators) and `HmiGateway` (status/events out)
- **Infrastructure:** adapters for Modbus (`ModbusPlcGateway`), the in-memory simulation, MQTT (`MqttHmiGateway`, `MqttCommandRouter`) and configuration

More details on the control logic, the I/O mapping and the MQTT contract are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Repository structure

```
control/                    Control system (Maven module)
hmi/                        Graphical and console HMI (Maven module)
run-control.ps1             Start the control system (simulation or real PLC)
run-hmi.ps1                 Start the HMI
mqtt-credentials.example.ps1  Template for broker settings
docs/                       Architecture notes
```

## Getting started

**Requirements:** Java 11 or newer, Maven, an MQTT broker (for example Mosquitto on `localhost:1883`).

> **Note on the Modbus library:** The Modbus adapter uses the course-provided `easymodbus` library, which is not available on Maven Central. To build the control module it has to be installed into the local Maven repository first (`mvn install` in the library project).

```bash
# Run the unit tests
cd control
mvn test

# Control system with the in-memory simulation and MQTT
mvn exec:java -Dexec.args="--sim --mqtt"

# Control system with an interactive console (type calls at runtime)
mvn exec:java -Dexec.args="--sim --interactive"

# Graphical HMI (separate terminal)
cd hmi
mvn exec:java -Dexec.mainClass=de.htwg.sysarch.hmi.HmiSwingApplication
```

On Windows, copy `mqtt-credentials.example.ps1` to `mqtt-credentials.ps1` and use `.\run-control.ps1` and `.\run-hmi.ps1`. Use `.\run-control.ps1 -Modbus` to drive a real PLC instead of the simulation.

Broker, PLC and topic are configurable via `-Dmqtt.host`, `-Dmqtt.port`, `-Dmqtt.baseTopic`, `-Dmodbus.host` and `-Dmodbus.port`.

## Documentation

- [Architecture, control logic and I/O mapping](docs/ARCHITECTURE.md)

## Team

- Control system: Ruben Kaiser, Lukas Rank
- Human machine interface: Yasin Maier, Mevlüt Atilla
