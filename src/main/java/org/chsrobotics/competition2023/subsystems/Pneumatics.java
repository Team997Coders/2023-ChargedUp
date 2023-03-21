/**
Copyright 2023 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.competition2023.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.lib.telemetry.Logger;

public class Pneumatics implements Subsystem {
    private static final Pneumatics instance = new Pneumatics();

    private final PneumaticHub pnHub = new PneumaticHub();

    private final String subdirString = "pneumatics";

    private final ArrayList<Integer> allocatedSolenoids = new ArrayList<>();

    private final Logger<Double> compressorCurrentLogger =
            new Logger<>("compressorCurrent_a", subdirString);
    private final Logger<Boolean> compressorActiveLogger =
            new Logger<>("compressorEnabled", subdirString);
    private final Logger<Double> solenoidCurrentLogger =
            new Logger<>("totalSolenoidCurrent_a", subdirString);
    private final Logger<Integer[]> allocatedSolenoidsLogger =
            new Logger<>("allocatedSolenoids", subdirString);

    private final Logger<Double> storagePressureLogger =
            new Logger<>("storagePressurePSI", subdirString);

    private final Logger<Boolean> pressureWarningLogger =
            new Logger<>("pressureWarning", subdirString);

    private Pneumatics() {
        register();

        pnHub.enableCompressorAnalog(
                Constants.SUBSYSTEM.PNEUMATICS.MIN_COMPRESSOR_PRESSURE_PSI,
                Constants.SUBSYSTEM.PNEUMATICS.MAX_PRESSURE_PSI);

        pnHub.clearStickyFaults();
    }

    public static Pneumatics getInstance() {
        return instance;
    }

    public Solenoid getSolenoid(int channel) {
        allocatedSolenoids.add(channel);
        return pnHub.makeSolenoid(channel);
    }

    public DoubleSolenoid getDoubleSolenoid(int forwardChannel, int reverseChannel) {
        allocatedSolenoids.addAll(List.of(forwardChannel, reverseChannel));

        return pnHub.makeDoubleSolenoid(forwardChannel, reverseChannel);
    }

    public double getPressurePSI() {
        return pnHub.getPressure(Constants.SUBSYSTEM.PNEUMATICS.ANALOG_PRESSURE_SENSOR_CHANNEL);
    }

    @Override
    public void periodic() {
        compressorActiveLogger.update(pnHub.getCompressor());
        compressorCurrentLogger.update(pnHub.getCompressorCurrent());
        solenoidCurrentLogger.update(pnHub.getSolenoidsTotalCurrent());
        allocatedSolenoidsLogger.update(allocatedSolenoids.toArray(new Integer[] {}));

        storagePressureLogger.update(getPressurePSI());

        pressureWarningLogger.update(
                getPressurePSI() <= Constants.SUBSYSTEM.PNEUMATICS.PRESSURE_WARNING_THRESHOLD);
    }
}
