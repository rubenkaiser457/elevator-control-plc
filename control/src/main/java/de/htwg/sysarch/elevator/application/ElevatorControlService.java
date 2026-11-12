package de.htwg.sysarch.elevator.application;

import de.htwg.sysarch.elevator.application.port.in.OperatorPanel;
import de.htwg.sysarch.elevator.application.port.out.HmiGateway;
import de.htwg.sysarch.elevator.application.port.out.PlcGateway;
import de.htwg.sysarch.elevator.domain.control.CycleResult;
import de.htwg.sysarch.elevator.domain.control.ElevatorController;
import de.htwg.sysarch.elevator.domain.control.RequestStore;
import de.htwg.sysarch.elevator.domain.model.Direction;
import de.htwg.sysarch.elevator.domain.model.HallCall;
import de.htwg.sysarch.elevator.domain.model.Level;
import de.htwg.sysarch.elevator.domain.control.Phase;
import de.htwg.sysarch.elevator.domain.model.LevelSensors;
import de.htwg.sysarch.elevator.domain.model.PlcInputs;
import de.htwg.sysarch.elevator.domain.model.PlcOutputs;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.logging.Logger;

/**
 * Orchestrates one control cycle and implements the {@link OperatorPanel} driving port.
 * Operator commands arrive from the HMI thread and are queued, then applied at the start
 * of the next cycle so the controller is only ever touched by the control-loop thread.
 */
public final class ElevatorControlService implements OperatorPanel {

    private static final Logger LOG = Logger.getLogger(ElevatorControlService.class.getName());

    /** Raw-bit diagnostics for the stop/door phases (door-safe diagnosis).
     *  Enabled by default; disable with -Delevator.diag=false when the log gets too chatty. */
    private static final boolean DIAG = !"false".equalsIgnoreCase(System.getProperty("elevator.diag", "true"));

    private final PlcGateway plc;
    private final HmiGateway hmi;
    private final ElevatorController controller = new ElevatorController();
    private final RequestStore requests = new RequestStore();
    private final ConcurrentLinkedQueue<Runnable> commands = new ConcurrentLinkedQueue<>();

    private volatile boolean resetPending;

    public ElevatorControlService(PlcGateway plc, HmiGateway hmi) {
        this.plc = plc;
        this.hmi = hmi;
    }

    /** One control cycle: apply queued commands, read PLC, step controller, write outputs, publish. */
    public void cycle(long now) throws Exception {
        Runnable command;
        while ((command = commands.poll()) != null) {
            command.run();
        }

        PlcInputs inputs = plc.read();
        CycleResult result = controller.step(inputs, requests, now);

        PlcOutputs outputs = result.outputs();
        if (resetPending) {
            outputs = outputs.withReset(true);
            resetPending = false;
        }
        plc.write(outputs);

        if (DIAG) {
            logStopDiagnostics(inputs, outputs, result);
        }

        result.events().forEach(hmi::log);
        hmi.publish(result.status());
    }

    /** One compact raw-bit line per cycle during the stop/door phases. This is the
     *  evidence trail for the door-safe condition (open question 3): which sensor
     *  bits are set when the PLC accepts vs. refuses a door-open request. Sensor
     *  bits are logged for ALL levels (al,sl,r,su,au per level) — during approach
     *  the interesting bits belong to the target, not the current level. */
    private void logStopDiagnostics(PlcInputs in, PlcOutputs out, CycleResult result) {
        Phase phase = result.status().phase();
        if (phase != Phase.CRAWLING && phase != Phase.DOOR_OPENING
                && phase != Phase.DECELERATING) {
            return;
        }
        StringBuilder sens = new StringBuilder();
        for (Level l : Level.values()) {
            LevelSensors s = in.at(l);
            if (sens.length() > 0) {
                sens.append(' ');
            }
            sens.append(l.number()).append(':')
                    .append(b(s.approachLower())).append(b(s.safetyLower()))
                    .append(b(s.reached())).append(b(s.safetyUpper()))
                    .append(b(s.approachUpper()));
        }
        LOG.info("DIAG " + phase
                + " lvl=" + result.status().currentLevel().number()
                + " dir=" + result.status().travelDirection()
                + " pos=" + result.status().positionMm()
                + " v=" + in.velocityCmPerS()
                + " sens[" + sens + "]"
                + " door[open=" + b(in.doorOpened()) + " closed=" + b(in.doorClosed()) + "]"
                + " motor[ready=" + b(in.motorReady()) + " on=" + b(in.motorOn()) + "]"
                + " out[dOpen=" + b(out.doorOpen()) + " dClose=" + b(out.doorClose())
                + " crawl=" + out.crawlCmPerS()
                + " drive=" + b(out.upV1() || out.upV2() || out.downV1() || out.downV2()) + "]");
    }

    private static String b(boolean v) {
        return v ? "1" : "0";
    }

    // ---------------------------------------------- OperatorPanel (HMI thread)

    @Override
    public void pressCabinButton(Level target) {
        commands.add(() -> requests.addCabinCall(target));
    }

    @Override
    public void pressHallButton(Level level, Direction direction) {
        commands.add(() -> requests.addHallCall(new HallCall(level, direction)));
    }

    @Override
    public void engageEmergencyStop() {
        commands.add(controller::engageEmergency);
    }

    @Override
    public void resetEmergencyStop() {
        commands.add(controller::resetEmergency);
    }

    @Override
    public void resetSimulation() {
        resetPending = true;
    }
}
