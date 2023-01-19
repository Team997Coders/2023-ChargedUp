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

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.lib.telemetry.Logger;

public class Pneumatics implements Subsystem {
    private static final Pneumatics instance = new Pneumatics();

    private final PneumaticHub pnHub = new PneumaticHub();

    private final String subdirString = "pneumatics";

    private final Logger<Double> compressorCurrentLogger =
            new Logger<>("compressorCurrent_a", subdirString);
    private final Logger<Boolean> compressorActiveLogger =
            new Logger<>("compressorEnabled", subdirString);
    private final Logger<Double> solenoidCurrentLogger =
            new Logger<>("solenoidCurrent_a", subdirString);

    private Pneumatics() {
        register();

        pnHub.enableCompressorDigital();
    }

    public static Pneumatics getInstance() {
        return instance;
    }

    public Solenoid getSolenoid(int channel) {
        return new Solenoid(PneumaticsModuleType.CTREPCM, channel);
    }

    @Override
    public void periodic() {
        compressorActiveLogger.update(pnHub.getCompressor());
        compressorCurrentLogger.update(pnHub.getCompressorCurrent());
        solenoidCurrentLogger.update(pnHub.getSolenoidsTotalCurrent());
    }
}
