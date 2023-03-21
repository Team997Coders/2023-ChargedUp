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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.lib.telemetry.Logger;

public class Claw implements Subsystem {
    private static final Claw instance = new Claw();

    private final Solenoid solenoid =
            Pneumatics.getInstance().getSolenoid(Constants.SUBSYSTEM.CLAW.SOLENOID_CHANNEL);

    private final String subdirString = "grabber";

    private final Logger<Boolean> closedLogger = new Logger<>("grabberClosed", subdirString);

    private boolean isClosed = false;

    private Claw() {
        register();
    }

    public static Claw getInstance() {
        return instance;
    }

    public void setSolenoid(boolean closed) {
        isClosed = closed;
    }

    public boolean isClosed() {
        return isClosed;
    }

    @Override
    public void periodic() {
        closedLogger.update(isClosed);

        if (isClosed) solenoid.set(!Constants.SUBSYSTEM.CLAW.SOLENOID_INVERTED);
        else solenoid.set(Constants.SUBSYSTEM.CLAW.SOLENOID_INVERTED);
    }
}
