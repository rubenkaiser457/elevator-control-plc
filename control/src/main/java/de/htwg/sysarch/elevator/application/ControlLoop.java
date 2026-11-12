package de.htwg.sysarch.elevator.application;

import java.util.logging.Level;
import java.util.logging.Logger;

/** Runs the control service cyclically at a fixed period (~100 ms, assignment §4). */
public final class ControlLoop {

    private static final Logger LOG = Logger.getLogger(ControlLoop.class.getName());

    private final ElevatorControlService service;
    private final long periodMillis;
    private volatile boolean running;
    private long overruns;

    public ControlLoop(ElevatorControlService service, long periodMillis) {
        this.service = service;
        this.periodMillis = periodMillis;
    }

    public void run() {
        running = true;
        long next = System.currentTimeMillis();
        while (running) {
            long now = System.currentTimeMillis();
            try {
                service.cycle(now);
            } catch (Exception e) {
                LOG.log(Level.SEVERE, "control cycle failed", e);
            }

            next += periodMillis;
            long sleep = next - System.currentTimeMillis();
            if (sleep > 0) {
                try {
                    Thread.sleep(sleep);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    running = false;
                }
            } else {
                next = System.currentTimeMillis(); // fell behind; resync
                // Chronic overruns mean the PLC I/O latency exceeds the cycle budget
                // (e.g. VPN instead of lab LAN): control still works — the controller
                // scales its per-cycle step sizes — but stops take visibly longer.
                if (++overruns == 1 || overruns % 100 == 0) {
                    LOG.warning("control cycle overran its " + periodMillis
                            + " ms budget (" + overruns + "× so far) — PLC I/O latency high;"
                            + " prefer running on the lab server for spec timing");
                }
            }
        }
    }

    public void stop() {
        running = false;
    }
}
