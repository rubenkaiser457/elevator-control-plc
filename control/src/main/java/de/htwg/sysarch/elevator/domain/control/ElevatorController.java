package de.htwg.sysarch.elevator.domain.control;

import de.htwg.sysarch.elevator.domain.model.Direction;
import de.htwg.sysarch.elevator.domain.model.DoorCommand;
import de.htwg.sysarch.elevator.domain.model.Drive;
import de.htwg.sysarch.elevator.domain.model.Level;
import de.htwg.sysarch.elevator.domain.model.LevelSensors;
import de.htwg.sysarch.elevator.domain.model.MotorCommand;
import de.htwg.sysarch.elevator.domain.model.PlcInputs;
import de.htwg.sysarch.elevator.domain.model.PlcOutputs;
import de.htwg.sysarch.elevator.domain.model.Speed;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * The elevator control state machine (assignment §1.6). Pure logic: a cycle is a
 * function of the current PLC inputs, the pending {@link RequestStore} and the
 * current time. It produces the {@link PlcOutputs} to write, an {@link ElevatorStatus}
 * for the HMI, and a list of {@link ElevatorEvent}s.
 *
 * <p>Deceleration chain (§1.6 (5), see docs/ARCHITECTURE.md): v2 while travelling, switch to
 * v1 at the target's approach sensor, to crawl at its safety sensor, and stop at its
 * reached sensor (the door-safe position).
 */
public final class ElevatorController {

    /** Minimum dwell time at a level (§1.6 (11)); door open/close time adds to this. */
    public static final long MIN_DWELL_MILLIS = 6_000L;

    /** If the PLC does not confirm the door open within this time, the cabin is assumed
     *  to be off the door-safe position → re-level instead of waiting forever. */
    public static final long DOOR_OPENING_TIMEOUT_MILLIS = 10_000L;

    /** While DOOR_OPENING, drop the POdopen coil for one cycle every this many cycles
     *  so the PLC sees a fresh rising edge. Live 2026-07-05 suggests the PLC samples
     *  its door-safe condition only on the edge of the open request: a refused stop
     *  was accepted ~10 s later at the SAME position — exactly when the watchdog
     *  re-level cycled the latched coil off and on again. Retrying every 2 s makes
     *  that recovery immediate instead of waiting for the 10 s watchdog. */
    public static final int DOOR_RETRY_CYCLES = 20;

    /** If the crawl/seek phase does not find the reached sensor within this time,
     *  something is physically wrong (stuck sensor, drive fault) → stop and alarm
     *  instead of hunting forever. */
    public static final long CRAWL_TIMEOUT_MILLIS = 30_000L;

    /** Fastest speed at which hitting the reached window counts as an arrival.
     *  0 = full standstill: live testing (2026-07-05) showed that opening the door
     *  at 1–2 cm/s lets the cabin creep out of the ±5 mm window before the PLC
     *  starts the door, causing an endless re-level loop. The cabin now settles
     *  (crawlCommand stops the motor inside the window) before the door opens. */
    public static final int MAX_STOP_VELOCITY_CM_PER_S = 0;

    /** Cycles of confirmed standstill inside the reached window before the door is
     *  requested. PIs_v is an INT in cm/s, so a reading of 0 can still mean up to
     *  ~0.9 cm/s of real creep — the PLC samples its door-safe condition when the
     *  open request arrives and refuses while moving (live 2026-07-05: first open
     *  attempt was refused, the re-issued one after the watchdog succeeded at the
     *  same position). Three cycles at commanded OFF ≈ 300 ms kill any residual
     *  creep before the request goes out. */
    public static final int SETTLE_CYCLES = 3;

    /** Extra creep cycles pushed deeper into the reached window after a door-open
     *  TIMEOUT (i.e. the PLC refused the settled stop for 10 s despite retry edges).
     *  Live 2026-07-06: every door requested while the reached bit was actually set
     *  opened fine — so the cabin stops at the first confirmed window hit and only
     *  nudges further on evidence of a real refusal, progressively per timeout. */
    public static final int NUDGE_CYCLES_PER_TIMEOUT = 2;
    public static final int MAX_NUDGE_CYCLES = 6;

    /** Input cycles ignored after a POreset pulse: the PLC re-initialises and
     *  reports garbage for a moment (live 2026-07-06: al+sl+r all set on level 4
     *  while the cabin physically sat at level 1) — pinning on those phantom bits
     *  corrupted the position fix. */
    public static final int POST_RESET_BLANK_CYCLES = 5;

    /** Level spacing (§4.2 assignment). Used for dead-reckoning position estimate. */
    private static final double LEVEL_HEIGHT_MM = 3500.0;
    /** Minimum distance from the stop position at which to leave V2 for V1 — the actual
     *  switch point is velocity-based (see brakeToV1DistanceMm) and usually earlier.
     *  Deliberately before the physical approach sensor (1.5 m) so the cabin is at v1
     *  when it crosses the approach window and the entry edge is reliably sampled. */
    private static final double APPROACH_DISTANCE_MM = 1700.0;

    /** Assumed PLC braking deceleration (cm/s²). Deliberately conservative — the real
     *  ramp looks closer to 25–30. Underestimating only makes the v1 phase start
     *  earlier and last longer; overestimating makes the cabin overshoot the level
     *  (observed on full-speed 1→4 runs before this was velocity-based). */
    private static final double BRAKE_DECEL_CM_PER_S2 = 20.0;
    private static final double V1_CM_PER_S = 10.0;
    /** Distance from the stop position at which the crawl phase begins (proactive
     *  dead-reckoning trigger in {@code applyDeadReckoningDecel}; the physical safety
     *  sensor at a fixed ±50 mm is the hard backstop regardless of this value).
     *
     *  <p>Raised 250 → 500 mm (2026-07-06 live session, level 4 motor error): over a
     *  laggy VPN cycle, a single poll can advance the cabin 200+ mm at V1 speed (one
     *  observed jump: 10205→10450 mm in one cycle) — with a 250 mm trigger distance
     *  that single stride can skip straight past the proactive check and land ON or
     *  past the physical safety sensor, so the crawl phase starts with essentially no
     *  runway to shed V1's 10 cm/s before the ±5 mm reached window (confirmed live:
     *  the cabin still read 7 cm/s at the reached hit). At a middle floor there is
     *  slack on both sides to recover; at level 4 (top, no slack) that residual speed
     *  carried the cabin into the shaft's mechanical end stop → motor error. 500 mm
     *  keeps a full cycle's worst-case stride of margin so the proactive crawl
     *  reliably starts before the safety sensor, giving the graded crawl speeds real
     *  distance to bring the velocity down first. */
    private static final double SAFETY_DISTANCE_MM = 500.0;

    /** How far past a terminal level the position estimate may run. The old hard clamp
     *  at the terminal made an overrun invisible (estimate said "at level 4", diff = 0)
     *  and the seek kept crawling INTO the shaft end until the PLC tripped a motor
     *  error (live 2026-07-06, twice). With this allowance the estimate keeps pointing
     *  back toward the shaft and the seek reverses correctly. */
    private static final double OVERTRAVEL_MM = 250.0;

    /** Seek hysteresis half-width. Inside this band around the target the estimate is
     *  no better than the accumulated integration error, so the seek does NOT trust
     *  it: it keeps its last direction until the estimate says it is this far past
     *  the target, then sweeps back — a bounded ±40 mm scan that always crosses the
     *  ±5 mm reached window. Freezing the motor at "estimate == target" instead
     *  parked the cabin in the sensor-free gap next to the window until the 30 s
     *  watchdog fired (simulated + live, 2026-07-06).
     *
     *  <p>Only applies while the estimate is "stale" (see {@link #TRUST_CYCLES}) —
     *  right after the reached sensor itself confirmed the position, this margin is
     *  not applied at all (see {@link #crawlCommand}). */
    private static final double SEEK_DEADBAND_MM = 40.0;

    /** Cycles after a confirmed reached-sensor hit during which the position estimate
     *  is trusted exactly (deadband-free correction) rather than falling back to the
     *  wide {@link #SEEK_DEADBAND_MM} scan (2026-07-06 live session, level 4): the
     *  cabin touches the reached window, coasts a few mm past it on momentum, and the
     *  old code re-entered the "blind search" hysteresis — which keeps driving in the
     *  PREVIOUS direction (away from a target it had just found) for up to 40 mm
     *  before reversing. At a middle floor that is a visible settle wobble; at level 4
     *  (top, no run-off room) that 40 mm push drove the cabin into the shaft's
     *  mechanical end stop and tripped a motor error. Right after a real sensor fix
     *  the estimate is accurate to a few mm, so no tolerance is needed at all — only
     *  once the cabin has searched for a while without finding the sensor again does
     *  the estimate become untrustworthy enough to warrant the wide scan. */
    private static final int TRUST_CYCLES = 5;

    private Level currentLevel = Level.LEVEL_1;
    private Direction travelDirection = Direction.NONE;
    private Phase phase = Phase.IDLE;
    private Level target;
    private long dwellEndsAt;
    private long doorOpeningDeadline;
    private long crawlingSince = -1;
    /** Cycles spent in DOOR_OPENING since the request; drives the retry pulsing. */
    private int doorOpeningCycles;
    /** Remaining creep cycles deeper into the reached window (set after door timeouts). */
    private int nudgeCyclesLeft;
    /** Last direction the crawl seek moved in (hysteresis memory for the ±40 mm scan). */
    private Direction lastSeek = Direction.NONE;
    /** Cycles since the CURRENT target's reached sensor was last observed active.
     *  Reset to 0 on a real hit, capped so it never overflows; see {@link #TRUST_CYCLES}. */
    private int cyclesSinceReachedPin = TRUST_CYCLES + 1;
    /** Which target {@link #cyclesSinceReachedPin} is tracking — lets {@link
     *  #crawlCommand} detect a newly set target and start it distrusted again. */
    private Level pinnedForTarget;
    /** Consecutive cycles at standstill inside the reached window (settle confirm). */
    private int settledCycles;
    /** Remaining input cycles to ignore after a POreset pulse (PLC re-init garbage). */
    private int blankCycles;
    /** Measured duration of the previous cycle in seconds (VPN can stretch the nominal
     *  100 ms to 300+; all per-cycle step sizes must scale with the real value). */
    private double cycleDtSec = 0.1;

    /** False until any level sensor has fixed the real cabin position (the PLC cabin
     *  does not necessarily start at level 1 — it keeps its position across runs). */
    private boolean positionKnown;
    private boolean homing;

    private boolean emergencyRequested;
    private boolean emergencyActive;
    private boolean errorLatched;

    /** Dead-reckoning position estimate in mm from Level 1 (corrected by level sensors). */
    private double estimatedPositionMm = 0.0;
    private long prevStepNow = -1;
    /** Crawl setpoint written in the previous cycle (cm/s) — used to keep the estimate
     *  moving when PIs_v truncates sub-1 cm/s motion to 0 during the fine seek. */
    private int lastCrawlCmdCmPerS;
    /** Previous cycle's inputs, for sensor edge detection (approach-window edge pinning). */
    private PlcInputs prevIn;

    // ------------------------------------------------- operator-side commands

    /** Cabin emergency stop (§1.6 (12)). */
    public void engageEmergency() {
        emergencyRequested = true;
    }

    /** Reset the emergency state; the elevator resumes toward the next scheduled stop (§1.6 (12)). */
    public void resetEmergency() {
        emergencyRequested = false;
    }

    public Phase phase() {
        return phase;
    }

    public Level currentLevel() {
        return currentLevel;
    }

    public Direction travelDirection() {
        return travelDirection;
    }

    // ------------------------------------------------------------- one cycle

    public CycleResult step(PlcInputs in, RequestStore requests, long now) {
        List<ElevatorEvent> events = new ArrayList<>();

        // Track the position even while halted (emergency/motor error): the cabin
        // still coasts out on the PLC's braking ramp (up to ~2 m from v2), and that
        // travel must not be lost from the dead-reckoning estimate.
        updatePositionEstimate(in, now);

        // Motor fault: pulse POreset for ONE cycle (§4.1 PIm_error). POreset also
        // resets the whole PLC simulation — holding it (the old behaviour) kept the
        // sim re-initialising under our motor commands (live 2026-07-05: erratic
        // velocity, door moving mid-travel). The reset teleports the cabin to its
        // initial position, so everything derived from the old position is
        // invalidated and the controller re-homes once the error clears.
        if (in.motorError()) {
            boolean firstErrorCycle = !errorLatched;
            if (firstErrorCycle) {
                events.add(ElevatorEvent.of(ElevatorEvent.Type.ALARM, currentLevel, "motor error"));
                errorLatched = true;
                positionKnown = false;
                homing = false;
                phase = Phase.IDLE;
                target = null;
                travelDirection = Direction.NONE;
                nudgeCyclesLeft = 0;
                settledCycles = 0;
                lastSeek = Direction.NONE;
            }
            // POreset restarts the whole PLC simulation; its inputs are garbage for
            // a few cycles afterwards (phantom sensor bits) — blank them out.
            blankCycles = POST_RESET_BLANK_CYCLES;
            return buildResult(MotorCommand.OFF, DoorCommand.NONE, firstErrorCycle, in, requests, events);
        }
        errorLatched = false;

        if (blankCycles > 0) {
            blankCycles--;
            return buildResult(MotorCommand.OFF, DoorCommand.NONE, false, in, requests, events);
        }

        // Emergency stop overrides everything (§1.6 (12)).
        if (emergencyRequested) {
            if (!emergencyActive) {
                emergencyActive = true;
                phase = Phase.EMERGENCY;
                events.add(ElevatorEvent.of(ElevatorEvent.Type.EMERGENCY_ENGAGED, currentLevel));
            }
            return buildResult(MotorCommand.OFF, DoorCommand.NONE, false, in, requests, events);
        }
        if (emergencyActive) {
            emergencyActive = false;
            events.add(ElevatorEvent.of(ElevatorEvent.Type.EMERGENCY_RESET, currentLevel));
            phase = (target != null && travelDirection != Direction.NONE) ? Phase.MOVING : Phase.IDLE;
        }

        // Startup calibration: the PLC cabin keeps its position across control restarts.
        // Until a sensor has fixed the real position, home downward at v1 — the first
        // sensor hit snaps the estimate (positionKnown), then crawl to the nearest level.
        if (!positionKnown) {
            homing = true;
            return buildResult(MotorCommand.drive(Direction.DOWN, Speed.V1),
                    DoorCommand.NONE, false, in, requests, events);
        }
        if (homing) {
            homing = false;
            target = nearestLevel(estimatedPositionMm);
            travelDirection = estimatedPositionMm < target.ordinal() * LEVEL_HEIGHT_MM
                    ? Direction.UP : Direction.DOWN;
            phase = Phase.DECELERATING;
            lastSeek = Direction.NONE;
        }

        advance(in, requests, now, events);

        return buildResult(deriveMotor(in), deriveDoor(in), false, in, requests, events);
    }

    // -------------------------------------------------------- state machine

    private void advance(PlcInputs in, RequestStore requests, long now, List<ElevatorEvent> events) {
        // Crawl watchdog: bounded seek time, then stop safely instead of hunting on.
        if (phase == Phase.CRAWLING) {
            if (crawlingSince < 0) {
                crawlingSince = now;
            } else if (now - crawlingSince > CRAWL_TIMEOUT_MILLIS) {
                crawlingSince = -1;
                phase = Phase.IDLE;
                target = null;
                travelDirection = Direction.NONE;
                events.add(ElevatorEvent.of(ElevatorEvent.Type.ALARM, currentLevel,
                        "leveling failed (crawl timeout), motor stopped"));
                return;
            }
        } else {
            crawlingSince = -1;
        }
        switch (phase) {
            case IDLE:
                handleIdle(in, requests, now, events);
                break;
            case MOVING:
            case DECELERATING:
            case CRAWLING:
                handleTravel(in, requests, now, events);
                break;
            case DOOR_OPENING:
                doorOpeningCycles++;
                if (in.doorOpened()) {
                    phase = Phase.DOOR_OPEN;
                    dwellEndsAt = now + MIN_DWELL_MILLIS;
                    nudgeCyclesLeft = 0;
                    events.add(ElevatorEvent.of(ElevatorEvent.Type.DOOR_OPENED, currentLevel));
                    // Clear the served requests only now that the door really opened —
                    // clearing on arrival lost them when the door was refused and the
                    // crawl watchdog gave up (cabin idled without ever serving).
                    Direction departing = requests.continueOrReverse(currentLevel, travelDirection);
                    requests.clearAt(currentLevel, travelDirection);
                    requests.clearAt(currentLevel, departing);
                } else if (!in.at(currentLevel).reached() && in.doorClosed()) {
                    // Cabin coasted out of the door-safe window before the door started
                    // opening: re-level immediately instead of waiting for the timeout.
                    // (Guarded on doorClosed — never move with the door in motion.)
                    phase = Phase.CRAWLING;
                    target = currentLevel;
                    settledCycles = 0;
                } else if (now >= doorOpeningDeadline) {
                    // Door never opened although the reached bit was set and the open
                    // request was re-issued with fresh edges: the PLC's door-safe band
                    // is apparently narrower here. Nudge progressively deeper into the
                    // window on each timeout so the retry never lands on the same spot.
                    phase = Phase.CRAWLING;
                    target = currentLevel;
                    settledCycles = 0;
                    nudgeCyclesLeft = Math.min(nudgeCyclesLeft + NUDGE_CYCLES_PER_TIMEOUT,
                            MAX_NUDGE_CYCLES);
                    events.add(ElevatorEvent.of(ElevatorEvent.Type.ALARM, currentLevel,
                            "door did not open, re-leveling"));
                }
                break;
            case DOOR_OPEN:
                if (now >= dwellEndsAt) {
                    phase = Phase.DOOR_CLOSING;
                    events.add(ElevatorEvent.of(ElevatorEvent.Type.DOOR_CLOSING, currentLevel));
                }
                break;
            case DOOR_CLOSING:
                if (in.doorClosed()) {
                    events.add(ElevatorEvent.of(ElevatorEvent.Type.DOOR_CLOSED, currentLevel));
                    departFrom(currentLevel, requests, events);
                }
                break;
            case EMERGENCY:
                /* handled in step() before advance() */
                break;
            default:
                break;
        }
    }

    private void handleIdle(PlcInputs in, RequestStore requests, long now, List<ElevatorEvent> events) {
        if (requests.hasAnyRequestAt(currentLevel)) {
            if (in.at(currentLevel).reached()) {
                beginDoorCycle(requests, now, events);
            } else {
                // The cabin is off the door-safe window although the request is "here":
                // re-level first — a door requested off-window is ignored by the PLC
                // and would just burn the 10 s timeout. If the cabin is actually FAR
                // away (stale level fix, e.g. before the first leveling), approach at
                // v1 with the normal sensor chain instead of crawling metres.
                target = currentLevel;
                settledCycles = 0;
                double diff = currentLevel.ordinal() * LEVEL_HEIGHT_MM - estimatedPositionMm;
                if (Math.abs(diff) > SAFETY_DISTANCE_MM) {
                    travelDirection = diff > 0 ? Direction.UP : Direction.DOWN;
                    phase = Phase.DECELERATING;
                } else {
                    phase = Phase.CRAWLING;
                }
            }
            return;
        }
        Direction d = requests.continueOrReverse(currentLevel, Direction.NONE);
        if (d != Direction.NONE) {
            startMoving(d, requests, events);
        }
    }

    private void handleTravel(PlcInputs in, RequestStore requests, long now, List<ElevatorEvent> events) {
        Level stop = (phase == Phase.MOVING)
                ? nextStop(requests, currentLevel, travelDirection)
                : target;

        if (stop == null) {
            departFrom(currentLevel, requests, events);
            return;
        }
        target = stop;

        LevelSensors s = in.at(stop);
        if (!s.reached()) {
            settledCycles = 0;
        }
        if (s.reached()) {
            currentLevel = stop;
            estimatedPositionMm = stop.ordinal() * LEVEL_HEIGHT_MM;
            boolean standstill = Math.abs(in.velocityCmPerS()) <= MAX_STOP_VELOCITY_CM_PER_S;
            if (nudgeCyclesLeft == 0 && standstill && ++settledCycles >= SETTLE_CYCLES) {
                settledCycles = 0;
                beginDoorCycle(requests, now, events);
            } else {
                // Not ready for the door yet: still nudging deeper into the window
                // (after a refused door), still moving, or standstill not yet
                // confirmed for SETTLE_CYCLES.
                if (!standstill) {
                    settledCycles = 0;
                }
                phase = Phase.CRAWLING;
            }
        } else if (s.safetyFrom(travelDirection)) {
            phase = Phase.CRAWLING;
        } else if (s.approachFrom(travelDirection) && phase == Phase.MOVING) {
            phase = Phase.DECELERATING;
        } else {
            // Dead-reckoning fallback: use estimated position when sensors are not seen
            // (e.g. Modbus polling missed a narrow sensor window at V2).
            boolean overshot = applyDeadReckoningDecel(stop, in.velocityCmPerS());
            if (overshot) {
                // Past the target without the reached sensor: re-level by crawling back
                // (deriveMotor picks the crawl direction from the position estimate).
                // Never fake an arrival — the PLC only opens the door at the safe position.
                phase = Phase.CRAWLING;
            }
        }
    }

    /**
     * Proactively decelerate based on dead-reckoning when PLC sensors are not (yet) seen.
     * Returns {@code true} when an overshoot is detected (caller re-levels).
     */
    private boolean applyDeadReckoningDecel(Level stop, int velocityCmPerS) {
        double targetMm = stop.ordinal() * LEVEL_HEIGHT_MM;
        double distMm = travelDirection == Direction.UP
                ? targetMm - estimatedPositionMm
                : estimatedPositionMm - targetMm;
        if (distMm < 0) {
            return true;  // overshot — caller re-levels by crawling back
        }
        // Leave V2 when the remaining distance no longer comfortably exceeds the
        // physical braking distance from the CURRENT speed down to v1 (v²-v1²)/2a.
        // Velocity-based, so a full-speed 1→4 run brakes much earlier than a short hop.
        double approachAt = Math.max(APPROACH_DISTANCE_MM,
                brakeToV1DistanceMm(velocityCmPerS) + SAFETY_DISTANCE_MM + 100.0);
        if (phase == Phase.MOVING && distMm <= approachAt) {
            phase = Phase.DECELERATING;
        }
        if ((phase == Phase.MOVING || phase == Phase.DECELERATING) && distMm <= SAFETY_DISTANCE_MM) {
            phase = Phase.CRAWLING;
        }
        return false;
    }

    /** Braking distance from the given speed down to v1, at the assumed PLC ramp. */
    private static double brakeToV1DistanceMm(int velocityCmPerS) {
        double v = Math.abs(velocityCmPerS);
        if (v <= V1_CM_PER_S) {
            return 0.0;
        }
        double distCm = (v * v - V1_CM_PER_S * V1_CM_PER_S) / (2.0 * BRAKE_DECEL_CM_PER_S2);
        return distCm * 10.0;
    }

    /** Decide direction and target after a stop; go IDLE if nothing remains (§1.6 (9),(10)). */
    private void departFrom(Level level, RequestStore requests, List<ElevatorEvent> events) {
        Direction d = requests.continueOrReverse(level, travelDirection);
        if (d == Direction.NONE) {
            travelDirection = Direction.NONE;
            target = null;
            phase = Phase.IDLE;
        } else {
            startMoving(d, requests, events);
        }
    }

    private void startMoving(Direction d, RequestStore requests, List<ElevatorEvent> events) {
        Level stop = nextStop(requests, currentLevel, d);
        if (stop == null) {
            travelDirection = Direction.NONE;
            target = null;
            phase = Phase.IDLE;
            return;
        }
        travelDirection = d;
        target = stop;
        phase = Phase.MOVING;
        nudgeCyclesLeft = 0;
        settledCycles = 0;
        lastSeek = Direction.NONE;
        events.add(ElevatorEvent.of(ElevatorEvent.Type.DEPARTURE, currentLevel, d.name()));
    }

    /** Arrive at {@code currentLevel}: request the door. The served requests are
     *  cleared once the door confirms open (see the DOOR_OPENING case in advance). */
    private void beginDoorCycle(RequestStore requests, long now, List<ElevatorEvent> events) {
        target = currentLevel;
        phase = Phase.DOOR_OPENING;
        doorOpeningCycles = 0;
        doorOpeningDeadline = now + DOOR_OPENING_TIMEOUT_MILLIS;
        events.add(ElevatorEvent.of(ElevatorEvent.Type.ARRIVAL, currentLevel));
        events.add(ElevatorEvent.of(ElevatorEvent.Type.DOOR_OPENING, currentLevel));
    }

    /** Nearest level in {@code direction} (strictly beyond {@code from}) the cabin must stop at. */
    private Level nextStop(RequestStore requests, Level from, Direction direction) {
        Level cur = from;
        while (true) {
            Optional<Level> next = cur.neighbour(direction);
            if (next.isEmpty()) {
                return null;
            }
            cur = next.get();
            if (requests.shouldStopAt(cur, direction)) {
                return cur;
            }
        }
    }

    private void updatePositionEstimate(PlcInputs in, long now) {
        // Integrate velocity to maintain a dead-reckoning position estimate.
        if (prevStepNow >= 0) {
            double dt = (now - prevStepNow) / 1000.0;
            if (dt > 0) {
                cycleDtSec = Math.max(0.02, Math.min(1.0, dt));
            }
            // PIs_v is an INT in cm/s, truncated toward zero: a reading of v means a
            // real speed in [v, v+1) going up. Integrating the raw value lags the real
            // position by up to 1 cm/s — ~10 cm over a full-shaft run, enough to brake
            // late and overrun the terminal. Integrate the interval midpoint instead.
            double v = in.velocityCmPerS();
            if (v > 0) {
                v += 0.5;
            } else if (v < 0) {
                v -= 0.5;
            } else if (in.motorOn() && lastCrawlCmdCmPerS != 0) {
                // Below 1 cm/s the INT-truncated PIs_v reads 0 although the cabin
                // moves — exactly during the fine seek, where drift matters most
                // (live 2026-07-06: the scan centre wandered ~40 mm off the real
                // position and circled the level for 30 s). The axis is speed-
                // controlled and tracks the crawl setpoint within its ramp, so use
                // a conservative share of the commanded speed instead of freezing.
                v = 0.7 * lastCrawlCmdCmPerS;
            }
            estimatedPositionMm += v * 10.0 * dt;
            double maxMm = (Level.COUNT - 1) * LEVEL_HEIGHT_MM;
            if (estimatedPositionMm < -OVERTRAVEL_MM) estimatedPositionMm = -OVERTRAVEL_MM;
            if (estimatedPositionMm > maxMm + OVERTRAVEL_MM) estimatedPositionMm = maxMm + OVERTRAVEL_MM;
        }
        prevStepNow = now;

        // Never pin on inputs while the PLC is in error/just after a reset (garbage
        // frames) or when the frame is physically impossible (more than one sensor
        // bit active in the whole shaft — the windows are all disjoint; live
        // 2026-07-06 saw al+sl+r simultaneously on level 4 right after POreset).
        if (in.motorError() || blankCycles > 0 || !plausibleSensorFrame(in)) {
            prevIn = in;
            return;
        }

        // Ground truth: every level sensor pins the estimate to a known offset, so the
        // estimate stays accurate even when individual narrow windows are missed, and
        // the real cabin position is picked up at startup (§ startup calibration).
        //
        // The narrow ±5 mm sensors (r/sl/su) pin to the window centre. The wide ±50 mm
        // approach windows pin on their ENTRY EDGE instead (position known to ~1 cycle
        // of travel, i.e. ±10 mm at v1) — centre-pinning them left up to 50 mm error,
        // which delayed the crawl switch enough to overshoot the level at v1.
        boolean movingUp = in.velocityCmPerS() > 0;
        boolean movingDown = in.velocityCmPerS() < 0;
        for (Level l : Level.values()) {
            LevelSensors s = in.at(l);
            LevelSensors p = prevIn == null ? LevelSensors.NONE : prevIn.at(l);
            double base = l.ordinal() * LEVEL_HEIGHT_MM;
            if (s.reached()) {
                currentLevel = l;
                estimatedPositionMm = base;
                positionKnown = true;
            } else if (s.safetyLower()) {
                estimatedPositionMm = base - 50.0;
                positionKnown = true;
            } else if (s.safetyUpper()) {
                estimatedPositionMm = base + 50.0;
                positionKnown = true;
            } else if (s.approachLower()) {
                if (!p.approachLower() && (movingUp || movingDown)) {
                    estimatedPositionMm = base - 1500.0 + (movingUp ? -50.0 : 50.0);
                } else if (!positionKnown) {
                    estimatedPositionMm = base - 1500.0;
                }
                positionKnown = true;
            } else if (s.approachUpper()) {
                if (!p.approachUpper() && (movingUp || movingDown)) {
                    estimatedPositionMm = base + 1500.0 + (movingUp ? -50.0 : 50.0);
                } else if (!positionKnown) {
                    estimatedPositionMm = base + 1500.0;
                }
                positionKnown = true;
            }
        }
        prevIn = in;
    }

    /** All shaft sensor windows are pairwise disjoint (nearest gap ≈ 35 mm between
     *  sl/r/su edges), so a physically possible frame has AT MOST ONE active sensor
     *  bit across all levels. More than one means the PLC is (re)initialising. */
    private static boolean plausibleSensorFrame(PlcInputs in) {
        int active = 0;
        for (Level l : Level.values()) {
            LevelSensors s = in.at(l);
            active += (s.approachLower() ? 1 : 0) + (s.safetyLower() ? 1 : 0)
                    + (s.reached() ? 1 : 0) + (s.safetyUpper() ? 1 : 0)
                    + (s.approachUpper() ? 1 : 0);
            if (active > 1) {
                return false;
            }
        }
        return true;
    }

    /** Level closest to the given shaft position. */
    private static Level nearestLevel(double positionMm) {
        int ordinal = (int) Math.round(positionMm / LEVEL_HEIGHT_MM);
        if (ordinal < 0) ordinal = 0;
        if (ordinal >= Level.COUNT) ordinal = Level.COUNT - 1;
        return Level.values()[ordinal];
    }

    /** Crawl toward the target with a graded speed profile: fast while far, slowing
     *  close in. The motor stops ONLY on the physically confirmed reached sensor —
     *  stopping on the estimate alone parked the cabin in the sensor-free 40 mm gap
     *  beside the reached window, where noise-fed corrections hunted around a phantom
     *  position forever (live PLC, 2026-07-05). Every sensor window passed while
     *  seeking re-pins the estimate, so the search converges.
     *
     *  <p>The reached check comes FIRST, before any direction logic: deciding the
     *  direction first deadlocked the FSM when the estimate sat exactly on the target
     *  (diff 0 → no direction → OFF before the reached bit was ever honoured; the
     *  cabin stood in the window while CRAWLING starved the 30 s watchdog —
     *  live 2026-07-06, twice on level 3). */
    private MotorCommand crawlCommand(PlcInputs in) {
        if (target != pinnedForTarget) {
            // A new target: the estimate has not been confirmed for THIS target yet,
            // so start distrusted regardless of how recently some other target's
            // sensor fired (otherwise a low leftover count could wrongly look "fresh"
            // for a brand new, still-unconfirmed seek).
            pinnedForTarget = target;
            cyclesSinceReachedPin = TRUST_CYCLES + 1;
        }
        if (target != null && in.at(target).reached()) {
            cyclesSinceReachedPin = 0;
            if (nudgeCyclesLeft > 0) {
                // A settled stop here was refused by the PLC: creep deeper into the
                // window so the retry lands on a different spot.
                nudgeCyclesLeft--;
                int v = in.velocityCmPerS();
                Direction creep = v > 0 ? Direction.UP
                        : (v < 0 ? Direction.DOWN : crawlDirection());
                if (creep == Direction.NONE) {
                    nudgeCyclesLeft = 0;
                    return MotorCommand.OFF;
                }
                return new MotorCommand(Drive.OFF, creep == Direction.UP ? 1 : -1);
            }
            return MotorCommand.OFF;    // in the window: settle; arrival follows at v=0
        }
        if (target == null) {
            return MotorCommand.OFF;    // nothing to seek toward: hold position
        }
        // Cap the counter (never used past TRUST_CYCLES+1, but avoid any theoretical
        // long-run overflow into a negative value that would look "fresh" again).
        if (cyclesSinceReachedPin <= TRUST_CYCLES) {
            cyclesSinceReachedPin++;
        }

        // Hysteresis scan: outside the deadband follow the estimate; inside it keep
        // the last direction (the estimate cannot tell which side of the window the
        // cabin is on). Crossing the deadband boundary flips the sign of diff, which
        // reverses the sweep — a bounded scan of ±SEEK_DEADBAND around the target.
        // It never runs away one-sided (the old travelDirection guess crawled into
        // the shaft end after a terminal overrun and tripped a motor error).
        //
        // The deadband only applies once the estimate has gone stale (no reached-pin
        // for more than TRUST_CYCLES): right after a real sensor hit it is trusted
        // exactly, so the cabin corrects back immediately instead of first coasting
        // further away in the old direction (see TRUST_CYCLES).
        double diff = target.ordinal() * LEVEL_HEIGHT_MM - estimatedPositionMm;
        double effectiveDeadband = cyclesSinceReachedPin <= TRUST_CYCLES ? 0.0 : SEEK_DEADBAND_MM;
        Direction d;
        if (Math.abs(diff) > effectiveDeadband) {
            d = diff > 0 ? Direction.UP : Direction.DOWN;
        } else if (lastSeek != Direction.NONE) {
            d = lastSeek;
        } else if (travelDirection != Direction.NONE) {
            d = travelDirection;
        } else {
            d = Direction.UP;
        }
        lastSeek = d;
        int speed = seekSpeed(Math.abs(diff));
        return new MotorCommand(Drive.OFF, d == Direction.UP ? speed : -speed);
    }

    /** Seek speed graded by remaining distance AND the measured cycle time, so one
     *  cycle's travel can never skip the ±5 mm reached window. The old fixed grades
     *  assumed 100 ms cycles ("1 cm/s = 1 mm per cycle"); over VPN a cycle is
     *  ~300 ms, tripling every step size.
     *
     *  <p>The whole scan region (deadband + margin) is capped at window-safe speed:
     *  the estimate there is off by up to the deadband, so the REAL window can sit
     *  anywhere in the region — a distance-graded 3 cm/s leg at 330 ms cycles
     *  stepped 10 mm per sample and hopped clean over the 10 mm window on every
     *  sweep (live 2026-07-06: startup seek circled level 2 for 2×30 s). */
    private int seekSpeed(double distMm) {
        double mmPerCycleAt1CmPerS = 10.0 * cycleDtSec;
        int windowSafe = (int) Math.max(1, Math.floor(3.0 / mmPerCycleAt1CmPerS));
        if (distMm > SEEK_DEADBAND_MM + 20.0) {
            return Math.min(5, windowSafe * 3);   // far out: hurry through the crawl zone
        }
        return Math.min(3, windowSafe);           // scan region: never skip the window
    }

    /** Crawl direction toward the target, based on the position estimate — reverses
     *  automatically when the cabin overshot the stop position (re-leveling).
     *  NONE when the estimate offers no usable side information. */
    private Direction crawlDirection() {
        if (target == null) {
            return travelDirection;
        }
        double diff = target.ordinal() * LEVEL_HEIGHT_MM - estimatedPositionMm;
        if (Math.abs(diff) < 1.0) {
            return Direction.NONE;
        }
        return diff > 0 ? Direction.UP : Direction.DOWN;
    }

    // ----------------------------------------------------------- output derivation

    private MotorCommand deriveMotor(PlcInputs in) {
        switch (phase) {
            case MOVING:
                return MotorCommand.drive(travelDirection, Speed.V2);
            case DECELERATING:
                return MotorCommand.drive(travelDirection, Speed.V1);
            case CRAWLING:
                return crawlCommand(in);
            default:
                return MotorCommand.OFF;
        }
    }

    private DoorCommand deriveDoor(PlcInputs in) {
        switch (phase) {
            case DOOR_OPENING:
                // Drop the coil for one cycle every DOOR_RETRY_CYCLES so a refused
                // request is retried with a fresh rising edge (see DOOR_RETRY_CYCLES).
                // Only while the door still reports closed — i.e. the request was
                // refused and the door never started moving; a door already in
                // motion keeps the coil held so the motion is never interrupted.
                if (in.doorClosed() && doorOpeningCycles > 0
                        && doorOpeningCycles % DOOR_RETRY_CYCLES == 0) {
                    return DoorCommand.NONE;
                }
                return DoorCommand.OPEN;
            case DOOR_CLOSING:
                return DoorCommand.CLOSE;
            case IDLE:
                return in.doorClosed() ? DoorCommand.NONE : DoorCommand.CLOSE;
            default:
                return DoorCommand.NONE;
        }
    }

    private CycleResult buildResult(MotorCommand motor, DoorCommand door, boolean reset,
                                    PlcInputs in, RequestStore requests, List<ElevatorEvent> events) {
        lastCrawlCmdCmPerS = motor.crawlCmPerS();
        PlcOutputs outputs = PlcOutputs.of(motor, door, reset);
        ElevatorStatus status = new ElevatorStatus(
                currentLevel, travelDirection, phase, in.doorState(), in.velocityCmPerS(),
                emergencyActive, in.motorError(),
                requests.cabinCalls(), requests.hallUpCalls(), requests.hallDownCalls(),
                (int) estimatedPositionMm);
        return new CycleResult(outputs, status, List.copyOf(events));
    }
}
