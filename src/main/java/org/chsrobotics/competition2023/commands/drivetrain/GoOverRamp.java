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
package org.chsrobotics.competition2023.commands.drivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.InertialMeasurement;
import org.chsrobotics.lib.math.UtilityMath;

public class GoOverRamp extends CommandBase {
    private enum State {
        DRIVE_TO,
        DRIVE_UP,
        FALLING,
        DRIVE_OFF;
    }

    private final Drivetrain drivetrain;

    private State state = State.DRIVE_TO;

    public GoOverRamp(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        state = State.DRIVE_TO;
    }

    @Override
    public void execute() {
        double gyroPitchDegrees =
                Units.radiansToDegrees(InertialMeasurement.getInstance().getPitchRadians());

        double gyroRateDPS =
                Units.radiansToDegrees(
                        InertialMeasurement.getInstance()
                                .getPitchAngularVelocityRadiansPerSecond());

        if (UtilityMath.epsilonEqualsAbsolute(
                gyroPitchDegrees, 0, Constants.COMMAND.GO_OVER_RAMP.BALANCED_TOLERANCE_DEGREES)) {
            state = State.DRIVE_TO;
        } else if (gyroPitchDegrees > Constants.COMMAND.GO_OVER_RAMP.BALANCED_TOLERANCE_DEGREES) {
            state = State.DRIVE_UP;
        } else if (Units.radiansToDegrees(gyroRateDPS) < 0) {
            state = State.FALLING;
        } else if (UtilityMath.epsilonEqualsAbsolute(
                gyroRateDPS,
                Constants.COMMAND.GO_OVER_RAMP.FALLING_RATE_TOLERANCE_DEGREES_PER_SECOND,
                gyroRateDPS)) {
            state = State.DRIVE_OFF;
        }

        double driveVoltage = Constants.COMMAND.GO_OVER_RAMP.CONTROL_INPUT_VOLTS;

        if (state == State.DRIVE_TO) {
            drivetrain.setLeftVoltages(driveVoltage);
            drivetrain.setRightVoltages(driveVoltage);
            if (gyroPitchDegrees > Constants.COMMAND.GO_OVER_RAMP.BALANCED_TOLERANCE_DEGREES) {
                state = State.DRIVE_UP;
            }
        } else if (state == State.DRIVE_UP) {
            drivetrain.setLeftVoltages(driveVoltage / 2);
            drivetrain.setRightVoltages(driveVoltage / 2);
            if (Units.radiansToDegrees(gyroRateDPS) < 0) {
                state = State.FALLING;
            }

        } else if (state == State.FALLING) {
            drivetrain.setLeftVoltages(0);
            drivetrain.setRightVoltages(0);
            if (UtilityMath.epsilonEqualsAbsolute(
                    gyroRateDPS,
                    Constants.COMMAND.GO_OVER_RAMP.FALLING_RATE_TOLERANCE_DEGREES_PER_SECOND,
                    gyroRateDPS)) {
                state = State.DRIVE_OFF;
            }
        } else if (state == State.DRIVE_OFF) {
            drivetrain.setLeftVoltages(0);
            drivetrain.setRightVoltages(0);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return state == State.DRIVE_OFF;
    }
}
