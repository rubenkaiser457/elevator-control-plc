package de.htwg.sysarch.elevator.infrastructure.modbus;

import de.re.easymodbus.modbusclient.ModbusClient;

import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * {@link ModbusConnection} implementation backed by the course-provided
 * course-provided easymodbus-maven library.
 *
 * The underlying {@link ModbusClient} is created once and reused across cycles.
 * A dropped TCP connection (read timeout / reset) otherwise permanently breaks
 * every subsequent cycle, so each operation transparently reconnects once and
 * retries before giving up; the caller ({@link ModbusPlcGateway}) still
 * propagates any exception it cannot recover from to the control loop.
 */
public final class EasyModbusConnection implements ModbusConnection {

    private static final Logger LOG = Logger.getLogger(EasyModbusConnection.class.getName());

    private final String host;
    private final int port;
    private final ModbusClient client;

    public EasyModbusConnection(String host, int port) {
        this.host = host;
        this.port = port;
        this.client = new ModbusClient(host, port);
    }

    @Override
    public void connect() throws Exception {
        client.Connect();
    }

    @Override
    public boolean[] readDiscreteInputs(int startAddress, int count) throws Exception {
        return withReconnect(() -> client.ReadDiscreteInputs(startAddress, count));
    }

    @Override
    public int[] readInputRegisters(int startAddress, int count) throws Exception {
        return withReconnect(() -> client.ReadInputRegisters(startAddress, count));
    }

    @Override
    public void writeCoil(int address, boolean value) throws Exception {
        withReconnect(() -> {
            client.WriteSingleCoil(address, value);
            return null;
        });
    }

    @Override
    public void writeCoils(int startAddress, boolean[] values) throws Exception {
        withReconnect(() -> {
            client.WriteMultipleCoils(startAddress, values);
            return null;
        });
    }

    @Override
    public void writeRegister(int address, int value) throws Exception {
        withReconnect(() -> {
            client.WriteSingleRegister(address, value);
            return null;
        });
    }

    private interface ModbusOp<T> {
        T run() throws Exception;
    }

    /** Runs {@code op}; on failure, reconnects once and retries before propagating. */
    private <T> T withReconnect(ModbusOp<T> op) throws Exception {
        try {
            return op.run();
        } catch (Exception first) {
            LOG.log(Level.WARNING, "Modbus operation failed, reconnecting to " + host + ":" + port, first);
            reconnect();
            return op.run();
        }
    }

    private void reconnect() {
        try {
            if (client.isConnected()) {
                client.Disconnect();
            }
        } catch (Exception ignored) {
            // best-effort close of the broken socket
        }
        try {
            client.Connect();
        } catch (Exception reconnectFailed) {
            LOG.log(Level.SEVERE, "Reconnect to " + host + ":" + port + " failed", reconnectFailed);
        }
    }

    @Override
    public void close() {
        try {
            if (client.isConnected()) {
                client.Disconnect();
            }
        } catch (Exception e) {
            // best-effort on shutdown
        }
    }

    @Override
    public String toString() {
        return "EasyModbusConnection{" + host + ":" + port + "}";
    }
}
