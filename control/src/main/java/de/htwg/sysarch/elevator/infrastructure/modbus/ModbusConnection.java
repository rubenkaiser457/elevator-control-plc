package de.htwg.sysarch.elevator.infrastructure.modbus;

/**
 * Thin abstraction over a Modbus/TCP master client, so the concrete library
 * (the course-provided easymodbus) stays swappable and the
 * gateway is unit-testable.
 */
public interface ModbusConnection extends AutoCloseable {

    void connect() throws Exception;

    boolean[] readDiscreteInputs(int startAddress, int count) throws Exception;

    int[] readInputRegisters(int startAddress, int count) throws Exception;

    void writeCoil(int address, boolean value) throws Exception;

    void writeCoils(int startAddress, boolean[] values) throws Exception;

    void writeRegister(int address, int value) throws Exception;

    @Override
    void close();
}
