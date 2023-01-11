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

public class Drivetrain implements Subsystem {
    private static Drivetrain instance = new Drivetrain();

    private Drivetrain() {}

    public static Drivetrain getInstance() {
        return instance;
    }

    public void setSimState(double leftMeters, double rightMeters) {
        if (!Robot.isReal()) {
        } else {
            HighLevelLogger.logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.logWarning("There might be sim code still running somewhere!");
        }
    }
}
