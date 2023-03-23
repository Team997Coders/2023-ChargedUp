package org.chsrobotics.competition2023.subsystems;

import com.fazecast.jSerialComm.*;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.telemetry.Logger;

import java.io.IOException;
import java.util.Arrays;
import java.util.stream.Collectors;

public class LedMatrix {
    public enum State {
        OFF(0x30),
        RED(0x31),
        BLUE(0x32),
        CONE(0x33),
        CUBE(0x34),
        MOVING(0x35);

        final int value;
        State(int value) {
            this.value = value;
        }
    }

    private static LedMatrix instance;
    public static LedMatrix getInstance() {
        if (instance == null) {
            instance = new LedMatrix();
        }
        return instance;
    }

    SerialPort serial;

    State state = State.OFF;
    public void setState(State state) {
        if (state != this.state) {
            this.state = state;
            if (connected) {
                sendState(state);
            }
        }
    }

    private void sendState(State state) {
        logger.update("Logging new state: "+state.value);
        // write thread
        new Thread(() -> {
            try {
                serial.getOutputStream().write(state.value);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }

    private final Logger<String> logger = new Logger<>("arduino", "ledMatrix");

    private boolean connected = false;

    private LedMatrix() {
        try {
            var ports = Arrays.stream(SerialPort.getCommPorts()).filter(port ->
                    port.getSystemPortName().startsWith("ttyACM")).collect(Collectors.toList());
            // first port should be correct I'm going to cry if it's not
            if (ports.isEmpty()) {
                logger.update("No arduino connected");
            }
            serial = ports.get(0);
            System.out.println(serial.getSystemPortName());
            serial.openPort();
            serial.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 1000, 0);
            // read thread
            new Thread(() -> {
                while (true) {
                    var avail = serial.bytesAvailable();
                    if (avail > 0) {
                        if (!connected) {
                            connected = true;
                            sendState(state);
                        }
                        byte[] bytes = new byte[avail];
                        serial.readBytes(bytes, avail);
                        String received = new String(bytes);
                        logger.update("Arduino: "+received);
                    }
                }
            }).start();
        } catch (Exception e) {
            HighLevelLogger.getInstance().logMessage(e.toString());
        }
    }
}
