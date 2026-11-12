package de.htwg.sysarch.elevator.infrastructure.modbus;

import de.htwg.sysarch.elevator.application.port.out.PlcGateway;
import de.htwg.sysarch.elevator.domain.model.Level;
import de.htwg.sysarch.elevator.domain.model.LevelSensors;
import de.htwg.sysarch.elevator.domain.model.PlcInputs;
import de.htwg.sysarch.elevator.domain.model.PlcOutputs;

import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.COIL_DOOR_CLOSE;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.COIL_DOOR_OPEN;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.COIL_DOWN_V1;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.COIL_DOWN_V2;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.COIL_RESET;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.COIL_UP_V1;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.COIL_UP_V2;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.DI_DOOR_CLOSED;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.DI_DOOR_OPENED;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.DI_MOTOR_ERROR;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.DI_MOTOR_ON;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.DI_MOTOR_READY;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.REG_AUFZUG_ID;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.REG_CRAWL_SELECT;
import static de.htwg.sysarch.elevator.infrastructure.modbus.ModbusIoMap.REG_VELOCITY;

/**
 * {@link PlcGateway} implementation over Modbus/TCP. Translates between the typed
 * domain snapshots and the raw Modbus addresses defined in {@link ModbusIoMap}.
 * The underlying client is abstracted by {@link ModbusConnection}.
 */
public final class ModbusPlcGateway implements PlcGateway {

    private static final java.util.logging.Logger LOG =
            java.util.logging.Logger.getLogger(ModbusPlcGateway.class.getName());

    // Diagnostic probing beyond the documented map (door-safe diagnosis):
    // -Dmodbus.probeRegs=<n> / -Dmodbus.probeBits=<n> additionally read the first n input
    // registers / discrete inputs every 10th cycle and log them raw. Off by default.
    private static final int PROBE_REGS = Integer.getInteger("modbus.probeRegs", 0);
    private static final int PROBE_BITS = Integer.getInteger("modbus.probeBits", 0);
    private long cycles;
    private boolean probeBroken;

    private final ModbusConnection connection;

    // Last written values, so unchanged reset/crawl writes can be skipped. The PLC
    // latches coil/register state, so re-writing an unchanged value is a no-op there;
    // skipping it saves Modbus round trips (matters over VPN, ~10-100 ms each).
    // ⚠️ The motor/door coil block is deliberately written EVERY cycle: skipping it
    // when unchanged (tried 2026-07-06) made the PLC cut the drive mid-travel after
    // a few seconds — it apparently expects a periodic command write as keep-alive
    // (reads alone don't count). Until 07-06 the per-cycle block write always
    // provided that heartbeat, which is why the reset/crawl caches never hurt.
    private Boolean lastReset;
    private Integer lastCrawl;

    public ModbusPlcGateway(ModbusConnection connection) {
        this.connection = connection;
    }

    @Override
    public void connect() throws Exception {
        connection.connect();
    }

    @Override
    public PlcInputs read() throws Exception {
        boolean[] di = connection.readDiscreteInputs(0, DI_MOTOR_ERROR + 1);
        int[] regs = connection.readInputRegisters(0, REG_VELOCITY + 1);

        cycles++;
        if ((PROBE_REGS > 0 || PROBE_BITS > 0) && !probeBroken && cycles % 10 == 1) {
            probe();
        }

        PlcInputs.Builder b = PlcInputs.builder();
        for (Level l : Level.values()) {
            b.sensors(l, new LevelSensors(
                    bit(di, ModbusIoMap.approachLower(l)),
                    bit(di, ModbusIoMap.safetyLower(l)),
                    bit(di, ModbusIoMap.reached(l)),
                    bit(di, ModbusIoMap.safetyUpper(l)),
                    bit(di, ModbusIoMap.approachUpper(l))));
        }
        return b.doorOpened(bit(di, DI_DOOR_OPENED))
                .doorClosed(bit(di, DI_DOOR_CLOSED))
                .motorReady(bit(di, DI_MOTOR_READY))
                .motorOn(bit(di, DI_MOTOR_ON))
                .motorError(bit(di, DI_MOTOR_ERROR))
                .velocityCmPerS((short) reg(regs, REG_VELOCITY))
                .aufzugId(reg(regs, REG_AUFZUG_ID))
                .build();
    }

    @Override
    public void write(PlcOutputs o) throws Exception {
        // Motor + door coils are contiguous (%QX1.0..1.5 → coils 8..13): one write.
        boolean[] block = new boolean[COIL_DOOR_OPEN - COIL_DOWN_V2 + 1];
        block[COIL_DOWN_V2 - COIL_DOWN_V2] = o.downV2();
        block[COIL_DOWN_V1 - COIL_DOWN_V2] = o.downV1();
        block[COIL_UP_V1 - COIL_DOWN_V2] = o.upV1();
        block[COIL_UP_V2 - COIL_DOWN_V2] = o.upV2();
        block[COIL_DOOR_CLOSE - COIL_DOWN_V2] = o.doorClose();
        block[COIL_DOOR_OPEN - COIL_DOWN_V2] = o.doorOpen();
        connection.writeCoils(COIL_DOWN_V2, block);

        if (lastReset == null || lastReset != o.reset()) {
            connection.writeCoil(COIL_RESET, o.reset());
            lastReset = o.reset();
        }
        if (lastCrawl == null || lastCrawl != o.crawlCmPerS()) {
            connection.writeRegister(REG_CRAWL_SELECT, o.crawlCmPerS());
            lastCrawl = o.crawlCmPerS();
        }
    }

    @Override
    public void close() {
        connection.close();
    }

    /** Reads the extended ranges and logs them raw; disables itself on the first failure
     *  (e.g. illegal-data-address from the slave) so the control cycle never suffers. */
    private void probe() {
        try {
            StringBuilder sb = new StringBuilder("PROBE");
            if (PROBE_REGS > 0) {
                int[] regs = connection.readInputRegisters(0, PROBE_REGS);
                sb.append(" regs=");
                for (int i = 0; i < regs.length; i++) {
                    sb.append(i == 0 ? "" : ",").append(i).append(':').append((short) regs[i]);
                }
            }
            if (PROBE_BITS > 0) {
                boolean[] bits = connection.readDiscreteInputs(0, PROBE_BITS);
                sb.append(" bits=");
                for (int byteIdx = 0; byteIdx * 8 < bits.length; byteIdx++) {
                    sb.append(byteIdx == 0 ? "" : " ").append(byteIdx).append(':');
                    for (int b = 0; b < 8 && byteIdx * 8 + b < bits.length; b++) {
                        sb.append(bits[byteIdx * 8 + b] ? '1' : '0');
                    }
                }
            }
            LOG.info(sb.toString());
        } catch (Exception e) {
            probeBroken = true;
            LOG.warning("register/bit probe failed, disabling probe: " + e.getMessage());
        }
    }

    private static boolean bit(boolean[] values, int index) {
        return index >= 0 && index < values.length && values[index];
    }

    private static int reg(int[] values, int index) {
        return index >= 0 && index < values.length ? values[index] : 0;
    }
}
