package de.htwg.sysarch.elevator.domain.control;

import de.htwg.sysarch.elevator.domain.model.Direction;
import de.htwg.sysarch.elevator.domain.model.Level;
import de.htwg.sysarch.elevator.domain.model.LevelSensors;
import de.htwg.sysarch.elevator.domain.model.PlcInputs;
import de.htwg.sysarch.elevator.domain.model.PlcOutputs;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ElevatorControllerTest {

    private final ElevatorController controller = new ElevatorController();
    private final RequestStore requests = new RequestStore();

    /** Fix the cabin position (startup calibration needs one sensor hit before travel). */
    private void startAtLevel1() {
        controller.step(sensor(Level.LEVEL_1, reached()), requests, -1);
        assertEquals(Level.LEVEL_1, controller.currentLevel());
    }

    @Test
    void fullStopSequenceForCabinCallAbove() {
        startAtLevel1();
        requests.addCabinCall(Level.LEVEL_3);

        // 1) idle with a request above → travel up at v2
        PlcOutputs o = controller.step(allSensorsClear(true), requests, 0).outputs();
        assertEquals(Phase.MOVING, controller.phase());
        assertTrue(o.upV2(), "should travel up at v2");

        // 2) target approach sensor → decelerate to v1
        o = controller.step(sensor(Level.LEVEL_3, approachLower()), requests, 10).outputs();
        assertEquals(Phase.DECELERATING, controller.phase());
        assertTrue(o.upV1());
        assertFalse(o.upV2());

        // 3) target safety sensor → crawl (coils off, crawl register set)
        o = controller.step(sensor(Level.LEVEL_3, safetyLower()), requests, 20).outputs();
        assertEquals(Phase.CRAWLING, controller.phase());
        assertEquals(de.htwg.sysarch.elevator.domain.model.MotorCommand.CRAWL_CM_PER_S, o.crawlCmPerS());
        assertFalse(o.upV1() || o.upV2());

        // 4) reached → centre into the window, confirm standstill, then the door
        //    opens (CENTER_CYCLES creep + SETTLE_CYCLES standstill confirmation)
        o = arriveAt(Level.LEVEL_3, 30);
        assertEquals(Phase.DOOR_OPENING, controller.phase());
        assertEquals(Level.LEVEL_3, controller.currentLevel());
        assertTrue(o.doorOpen());

        // 5) door reports open → dwell starts
        controller.step(doorState(true, false), requests, 1_000);
        assertEquals(Phase.DOOR_OPEN, controller.phase());

        // 6) after the minimum dwell (6 s) → door closes
        o = controller.step(doorState(false, false), requests,
                1_000 + ElevatorController.MIN_DWELL_MILLIS + 1).outputs();
        assertEquals(Phase.DOOR_CLOSING, controller.phase());
        assertTrue(o.doorClose());

        // 7) door closed, no further requests → idle
        controller.step(doorState(false, true), requests, 9_000);
        assertEquals(Phase.IDLE, controller.phase());
    }

    @Test
    void emergencyStopHaltsMotorAndResumesAfterReset() {
        startAtLevel1();
        requests.addCabinCall(Level.LEVEL_4);
        controller.engageEmergency();

        var result = controller.step(allSensorsClear(true), requests, 0);
        assertEquals(Phase.EMERGENCY, controller.phase());
        assertTrue(result.status().emergencyActive());
        assertFalse(anyMotorCoil(result.outputs()));

        controller.resetEmergency();
        var resumed = controller.step(allSensorsClear(true), requests, 1);
        assertFalse(resumed.status().emergencyActive());
        assertEquals(Phase.MOVING, controller.phase(), "should resume toward the scheduled stop");
        assertTrue(resumed.outputs().upV2());
    }

    @Test
    void motorErrorPulsesResetAndStopsMotor() {
        requests.addCabinCall(Level.LEVEL_4);

        PlcInputs.Builder b = PlcInputs.builder();
        for (Level l : Level.values()) {
            b.sensors(l, LevelSensors.NONE);
        }
        PlcOutputs o = controller.step(b.motorError(true).build(), requests, 0).outputs();

        assertTrue(o.reset(), "POreset should be pulsed on motor error");
        assertFalse(anyMotorCoil(o));
    }

    @Test
    void reversesDirectionToServeRequestBelowAfterStop() {
        // position the cabin at level 2 (idle, no requests yet)
        controller.step(sensor(Level.LEVEL_2, reached()), requests, -1);
        assertEquals(Level.LEVEL_2, controller.currentLevel());

        requests.addCabinCall(Level.LEVEL_4); // above → served first
        requests.addCabinCall(Level.LEVEL_1); // below → must be served afterwards (reversal)

        // travel up to level 4 and complete the door cycle
        controller.step(allSensorsClear(true), requests, 0);
        controller.step(sensor(Level.LEVEL_4, approachLower()), requests, 10);
        controller.step(sensor(Level.LEVEL_4, safetyLower()), requests, 20);
        arriveAt(Level.LEVEL_4, 30);
        controller.step(doorState(true, false), requests, 1_000);
        controller.step(doorState(false, false), requests, 1_000 + ElevatorController.MIN_DWELL_MILLIS + 1);
        PlcOutputs o = controller.step(doorState(false, true), requests, 9_000).outputs();

        // no more requests above → reverse downward to serve level 1
        assertEquals(Direction.DOWN, controller.travelDirection());
        assertEquals(Phase.MOVING, controller.phase());
        assertTrue(o.downV2());
    }

    @Test
    void coastOutOfWindowRecoversWithoutDeadlock() {
        // Idle at level 3, request there → door requested; the cabin then drifts out
        // of the window (travelDirection is NONE in this state). The old direction-
        // first crawl logic deadlocked here — diff 0 → no direction → motor OFF
        // before the reached bit was honoured — until the 30 s watchdog fired
        // (live PLC 2026-07-06, twice on level 3).
        controller.step(sensor(Level.LEVEL_3, reached()), requests, -1);
        requests.addCabinCall(Level.LEVEL_3);
        controller.step(sensor(Level.LEVEL_3, reached()), requests, 0);
        assertEquals(Phase.DOOR_OPENING, controller.phase());

        // drift out: r drops, door still closed → immediate re-level
        controller.step(allSensorsClear(true), requests, 100);
        assertEquals(Phase.CRAWLING, controller.phase());

        // back in the window at standstill → settle and re-request within a few cycles
        long t = 200;
        for (int i = 0; i < ElevatorController.SETTLE_CYCLES + 2
                && controller.phase() != Phase.DOOR_OPENING; i++, t += 100) {
            controller.step(sensor(Level.LEVEL_3, reached()), requests, t);
        }
        assertEquals(Phase.DOOR_OPENING, controller.phase(),
                "must re-request the door, not deadlock in CRAWLING");
    }

    @Test
    void terminalOvershootSeeksBackInsteadOfIntoShaftEnd() {
        // Position known at level 3; call to level 4 (terminal — no sensor above).
        controller.step(sensor(Level.LEVEL_3, reached()), requests, 0);
        requests.addCabinCall(Level.LEVEL_4);
        controller.step(allSensorsClear(true), requests, 100);
        assertEquals(Phase.MOVING, controller.phase());

        // Climb fast past level 4 with no sensors visible: the estimate must keep
        // pointing back toward the shaft (the old clamp froze it AT level 4, diff 0,
        // and the seek crawled INTO the shaft end until the PLC tripped a motor
        // error — live PLC 2026-07-06, twice).
        long t = 100;
        for (int i = 0; i < 40; i++) {
            t += 100;
            controller.step(velocity(100), requests, t);
        }
        PlcOutputs o = controller.step(velocity(0), requests, t + 100).outputs();
        assertEquals(Phase.CRAWLING, controller.phase());
        assertTrue(o.crawlCmPerS() < 0, "must seek DOWN toward level 4, not up into the end");
    }

    @Test
    void idleDoorRequestOffWindowRelevelsFirst() {
        controller.step(sensor(Level.LEVEL_3, reached()), requests, -1);
        // request arrives while the reached bit is NOT set (cabin drifted while idle):
        // the PLC ignores off-window door commands, so re-level instead of requesting.
        requests.addCabinCall(Level.LEVEL_3);
        controller.step(allSensorsClear(true), requests, 0);
        assertEquals(Phase.CRAWLING, controller.phase(), "must re-level before the door");
    }

    @Test
    void phantomSensorFrameAfterResetIsIgnored() {
        // Motor error → POreset pulse; the PLC re-initialises and briefly reports
        // impossible frames (several sensor bits at once — live PLC 2026-07-06).
        controller.step(sensor(Level.LEVEL_2, reached()), requests, -1);
        requests.addCabinCall(Level.LEVEL_4);

        PlcInputs.Builder err = PlcInputs.builder();
        for (Level l : Level.values()) {
            err.sensors(l, LevelSensors.NONE);
        }
        controller.step(err.motorError(true).build(), requests, 0);

        // phantom frame: reached active on levels 1 AND 4 simultaneously
        PlcInputs phantom = twoReached(Level.LEVEL_1, Level.LEVEL_4);
        for (int i = 1; i <= ElevatorController.POST_RESET_BLANK_CYCLES; i++) {
            PlcOutputs o = controller.step(phantom, requests, i * 100L).outputs();
            assertFalse(anyMotorCoil(o), "blanked cycles must keep the motor off");
        }
        assertEquals(Level.LEVEL_2, controller.currentLevel(),
                "phantom bits must not move the level fix");

        // After the blanking window, position is unknown → re-homes downward.
        PlcOutputs o = controller.step(allSensorsClear(true), requests, 1_000).outputs();
        assertTrue(o.downV1(), "must re-home after reset instead of trusting phantom bits");
    }

    @Test
    void refusedDoorOpenIsRetriedWithFreshRisingEdge() {
        // Arrive at level 1 with a request there → DOOR_OPENING, but the PLC
        // refuses: the door stays closed. The open coil must be dropped for one
        // cycle every DOOR_RETRY_CYCLES so the PLC sees a fresh rising edge.
        startAtLevel1();
        requests.addCabinCall(Level.LEVEL_1);
        controller.step(sensor(Level.LEVEL_1, reached()), requests, 0);
        assertEquals(Phase.DOOR_OPENING, controller.phase());

        int pulses = 0;
        long t = 100;
        for (int i = 1; i <= ElevatorController.DOOR_RETRY_CYCLES + 2; i++, t += 100) {
            PlcOutputs o = controller.step(sensor(Level.LEVEL_1, reached()), requests, t).outputs();
            assertEquals(Phase.DOOR_OPENING, controller.phase());
            assertFalse(o.doorClose(), "must never command close while opening");
            if (!o.doorOpen()) {
                pulses++;
                assertEquals(ElevatorController.DOOR_RETRY_CYCLES, i,
                        "coil must drop exactly at the retry interval");
            }
        }
        assertEquals(1, pulses, "exactly one one-cycle drop per retry interval");

        // A door already in motion (no longer closed) must NOT be interrupted.
        for (int i = 0; i < ElevatorController.DOOR_RETRY_CYCLES + 2; i++, t += 100) {
            PlcOutputs o = controller.step(doorState(false, false), requests, t).outputs();
            if (controller.phase() != Phase.DOOR_OPENING) {
                break;
            }
            assertTrue(o.doorOpen(), "moving door keeps the open coil held");
        }
    }

    @Test
    void emergencyDuringTravelStopsAndResumesSameDirection() {
        startAtLevel1();
        requests.addCabinCall(Level.LEVEL_4);

        PlcOutputs moving = controller.step(allSensorsClear(true), requests, 0).outputs();
        assertTrue(moving.upV2());

        controller.engageEmergency();
        PlcOutputs halted = controller.step(allSensorsClear(true), requests, 10).outputs();
        assertEquals(Phase.EMERGENCY, controller.phase());
        assertFalse(anyMotorCoil(halted));

        controller.resetEmergency();
        PlcOutputs resumed = controller.step(allSensorsClear(true), requests, 20).outputs();
        assertEquals(Phase.MOVING, controller.phase());
        assertTrue(resumed.upV2(), "should resume travelling up toward level 4");
    }

    // ------------------------------------------------------------- fixtures

    /** Step with the level's reached sensor active until the door cycle begins
     *  (covers the SETTLE_CYCLES standstill confirmation). */
    private PlcOutputs arriveAt(Level level, long startTime) {
        PlcOutputs o = null;
        long t = startTime;
        int maxSteps = ElevatorController.SETTLE_CYCLES + 4;
        for (int i = 0; i < maxSteps && controller.phase() != Phase.DOOR_OPENING; i++, t += 100) {
            o = controller.step(sensor(level, reached()), requests, t).outputs();
        }
        return o;
    }

    private static boolean anyMotorCoil(PlcOutputs o) {
        return o.upV2() || o.upV1() || o.downV1() || o.downV2();
    }

    private static LevelSensors approachLower() {
        return new LevelSensors(true, false, false, false, false);
    }

    private static LevelSensors safetyLower() {
        return new LevelSensors(false, true, false, false, false);
    }

    private static LevelSensors reached() {
        return new LevelSensors(false, false, true, false, false);
    }

    private static PlcInputs allSensorsClear(boolean doorClosed) {
        return doorState(false, doorClosed);
    }

    private static PlcInputs doorState(boolean opened, boolean closed) {
        PlcInputs.Builder b = PlcInputs.builder();
        for (Level l : Level.values()) {
            b.sensors(l, LevelSensors.NONE);
        }
        return b.doorOpened(opened).doorClosed(closed).motorReady(true).build();
    }

    private static PlcInputs sensor(Level level, LevelSensors sensors) {
        PlcInputs.Builder b = PlcInputs.builder();
        for (Level l : Level.values()) {
            b.sensors(l, l == level ? sensors : LevelSensors.NONE);
        }
        return b.doorClosed(true).motorReady(true).build();
    }

    /** All sensors clear, door closed, moving at the given velocity (dead reckoning). */
    private static PlcInputs velocity(int cmPerS) {
        PlcInputs.Builder b = PlcInputs.builder();
        for (Level l : Level.values()) {
            b.sensors(l, LevelSensors.NONE);
        }
        return b.doorClosed(true).motorReady(true).velocityCmPerS(cmPerS).build();
    }

    /** Physically impossible frame: the reached bit active on two levels at once. */
    private static PlcInputs twoReached(Level a, Level b) {
        PlcInputs.Builder bld = PlcInputs.builder();
        for (Level l : Level.values()) {
            bld.sensors(l, (l == a || l == b) ? reached() : LevelSensors.NONE);
        }
        return bld.doorClosed(true).motorReady(true).build();
    }
}
