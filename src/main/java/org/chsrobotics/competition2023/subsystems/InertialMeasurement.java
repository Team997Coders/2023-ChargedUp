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

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Robot;
import org.chsrobotics.lib.telemetry.HighLevelLogger;

public class InertialMeasurement implements Subsystem {
    private static final InertialMeasurement instance = new InertialMeasurement();

    private InertialMeasurement() {
        register();
    }

    public static InertialMeasurement getInstance() {
        return instance;
    }

    public void setSimState(
            double pitch, double yaw, double roll, double xAccel, double yAccel, double zAccel) {
        if (!Robot.isReal()) {
        } else {
            HighLevelLogger.getInstance()
                    .logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.getInstance()
                    .logWarning("There might be sim code still running somewhere!");
        }
    }
}
