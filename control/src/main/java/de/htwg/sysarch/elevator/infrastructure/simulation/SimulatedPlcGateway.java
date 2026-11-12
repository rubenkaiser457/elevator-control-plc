package de.htwg.sysarch.elevator.infrastructure.simulation;

import de.htwg.sysarch.elevator.application.port.out.PlcGateway;
import de.htwg.sysarch.elevator.domain.model.Level;
import de.htwg.sysarch.elevator.domain.model.LevelSensors;
import de.htwg.sysarch.elevator.domain.model.PlcInputs;
import de.htwg.sysarch.elevator.domain.model.PlcOutputs;

/**
 * Lightweight in-memory model of the lab PLC for offline development and demos.
 *
 * <p>NOT a physically accurate twin, but it reproduces the behaviours the control
 * logic is tuned against (confirmed in live PLC testing):
 * <ul>
 *   <li>the cabin position and the approach → safety → reached sensor chain,</li>
 *   <li>the motor <b>acceleration/deceleration ramp</b> (~25 cm/s²) — commanding a
 *       speed change does not take effect instantly, so the cabin coasts past the
 *       command point exactly like the real drive (this is what made the cabin
 *       overshoot levels before the controller's velocity-based braking),</li>
 *   <li>the <b>door-safe interlock</b>: the PLC refuses to open the door unless the
 *       cabin is inside a level's reached window (an open command elsewhere is
 *       silently ignored — the failure mode behind the DOOR_OPENING stall bug),</li>
 *   <li>door open/close travel time.</li>
 * </ul>
 * Replace with the real {@code ModbusPlcGateway} for hardware runs.
 */
public final class SimulatedPlcGateway implements PlcGateway {

    private static final int LEVEL_HEIGHT_MM = 3500;
    private static final int TOP_MM = (Level.COUNT - 1) * LEVEL_HEIGHT_MM;

    // Sensor offsets relative to a level (al, sl, r, su, au) and their tolerances —
    // the REAL tolerances from assignment §6.1 (approach ±50 mm, safety/reached ±5 mm).
    // They must not be widened "for safety": bigger safety windows overlap around the
    // level, and the controller's sensor pinning then receives contradictory position
    // fixes (observed as an endless ±crawl oscillation just above the target level).
    // Skipping a narrow window between polls is already handled by crossed().
    private static final int[] OFFSET_MM = {-1500, -50, 0, 50, 1500};
    private static final int APPROACH_TOL_MM = 50;
    private static final int SAFETY_TOL_MM = 5;
    private static final int REACHED_TOL_MM = 5;

    /** Drive ramp: actual velocity approaches the commanded one at this rate. The
     *  real PLC ramp was observed at ~25–30 cm/s² (the controller conservatively
     *  assumes 20, see ElevatorController.BRAKE_DECEL_CM_PER_S2). */
    private static final double RAMP_MM_PER_S2 = 250.0;

    private static final long DOOR_TRAVEL_MS = 1500;

    private enum Door {CLOSED, OPENING, OPEN, CLOSING}

    private double positionMm;          // 0 = level 1
    private double prevPositionMm;      // position at start of last cycle (for crossed-sensor detection)
    private double velocityMmPerS;      // actual (ramped) velocity, mm/s
    private long lastMillis = -1;
    private PlcOutputs last = PlcOutputs.IDLE;
    private Door door = Door.CLOSED;
    private long doorTimerEndsAt;

    public SimulatedPlcGateway() {
        // The real PLC cabin keeps its position across control restarts; let tests
        // start mid-shaft to exercise the startup calibration/homing path.
        positionMm = clampToShaft(Double.parseDouble(
                System.getProperty("sim.startPositionMm", "0")));
        prevPositionMm = positionMm;
    }

    private static double clampToShaft(double mm) {
        return Math.max(0, Math.min(mm, TOP_MM));
    }

    @Override
    public void connect() {
        // nothing to connect for the simulation
    }

    @Override
    public void close() {
        // nothing to release
    }

    @Override
    public void write(PlcOutputs outputs) {
        this.last = outputs;
    }

    @Override
    public PlcInputs read() {
        long now = System.currentTimeMillis();
        double dt = lastMillis < 0 ? 0.0 : (now - lastMillis) / 1000.0;
        lastMillis = now;

        // Ramp the actual velocity toward the commanded one, then integrate the
        // position with the average velocity over the step (trapezoid).
        double commanded = commandedVelocityMmPerS(last);
        double prevV = velocityMmPerS;
        double dv = commanded - velocityMmPerS;
        double maxStep = RAMP_MM_PER_S2 * dt;
        velocityMmPerS = Math.abs(dv) <= maxStep ? commanded : velocityMmPerS + Math.copySign(maxStep, dv);

        prevPositionMm = positionMm;
        positionMm += (prevV + velocityMmPerS) / 2.0 * dt;
        if (positionMm <= 0) {                    // shaft end stops
            positionMm = 0;
            if (velocityMmPerS < 0) velocityMmPerS = 0;
        } else if (positionMm >= TOP_MM) {
            positionMm = TOP_MM;
            if (velocityMmPerS > 0) velocityMmPerS = 0;
        }
        updateDoor(now);

        PlcInputs.Builder b = PlcInputs.builder();
        for (Level l : Level.values()) {
            double levelMm = l.ordinal() * (double) LEVEL_HEIGHT_MM;
            // A sensor fires if the cabin is currently in its window OR passed through it this cycle.
            // This prevents fast movement from skipping narrow sensor windows between polling cycles.
            b.sensors(l, new LevelSensors(
                    crossed(levelMm + OFFSET_MM[0], APPROACH_TOL_MM),
                    crossed(levelMm + OFFSET_MM[1], SAFETY_TOL_MM),
                    crossed(levelMm + OFFSET_MM[2], REACHED_TOL_MM),
                    crossed(levelMm + OFFSET_MM[3], SAFETY_TOL_MM),
                    crossed(levelMm + OFFSET_MM[4], APPROACH_TOL_MM)));
        }
        return b.doorOpened(door == Door.OPEN)
                .doorClosed(door == Door.CLOSED)
                .motorReady(true)
                .motorOn(Math.abs(velocityMmPerS) > 0.01)
                .velocityCmPerS((int) (velocityMmPerS / 10.0))
                .build();
    }

    private static double commandedVelocityMmPerS(PlcOutputs o) {
        if (o.upV2()) {
            return 1000;
        }
        if (o.upV1()) {
            return 100;
        }
        if (o.downV1()) {
            return -100;
        }
        if (o.downV2()) {
            return -1000;
        }
        return o.crawlCmPerS() * 10; // crawl register: cm/s → mm/s
    }

    private void updateDoor(long now) {
        switch (door) {
            case CLOSED:
                // Door-safe interlock: like the real PLC, an open command is ignored
                // unless the cabin is inside a level's reached window.
                if (last.doorOpen() && atDoorSafePosition()) {
                    door = Door.OPENING;
                    doorTimerEndsAt = now + DOOR_TRAVEL_MS;
                }
                break;
            case OPENING:
                if (now >= doorTimerEndsAt) {
                    door = Door.OPEN;
                }
                break;
            case OPEN:
                if (last.doorClose()) {
                    door = Door.CLOSING;
                    doorTimerEndsAt = now + DOOR_TRAVEL_MS;
                }
                break;
            case CLOSING:
                if (last.doorOpen()) {
                    door = Door.OPENING;
                    doorTimerEndsAt = now + DOOR_TRAVEL_MS;
                } else if (now >= doorTimerEndsAt) {
                    door = Door.CLOSED;
                }
                break;
        }
    }

    /** True while the cabin sits inside some level's reached (door-safe) window. */
    private boolean atDoorSafePosition() {
        for (Level l : Level.values()) {
            if (Math.abs(positionMm - l.ordinal() * (double) LEVEL_HEIGHT_MM) <= REACHED_TOL_MM) {
                return true;
            }
        }
        return false;
    }

    /** True if the cabin is currently in the sensor window OR moved through it this cycle. */
    private boolean crossed(double sensorMm, int tolerance) {
        return Math.max(prevPositionMm, positionMm) >= sensorMm - tolerance
                && Math.min(prevPositionMm, positionMm) <= sensorMm + tolerance;
    }
}
