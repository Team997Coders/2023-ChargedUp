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
package org.chsrobotics.competition2023.commands.arm;

import edu.wpi.first.math.util.Units;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.math.UtilityMath;

public class SnapToRightAngle extends ArmSetpointControl {
    private final Arm arm;

    private final double setpoint;

    public SnapToRightAngle(Arm arm) {
        super(arm, Math.PI / 2, getSetpoint(arm.getDistalAngleRadians()));

        this.arm = arm;

        this.setpoint = getSetpoint(arm.getDistalAngleRadians());
    }

    private static double getSetpoint(double currentPosition) {
        return (Math.abs(UtilityMath.smallestAngleRadiansBetween(currentPosition, Math.PI / 2))
                        < Math.abs(
                                UtilityMath.smallestAngleDegreesBetween(
                                        currentPosition, (3 * Math.PI) / 2)))
                ? (Math.PI / 2)
                : ((3 * Math.PI) / 2);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(arm.getLocalAngleRadians() - Math.PI / 2) < Units.degreesToRadians(2))
                && (Math.abs(arm.getLocalAngleRadians() - setpoint) < Units.degreesToRadians(2));
    }
}
