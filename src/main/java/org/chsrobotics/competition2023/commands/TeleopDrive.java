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
package org.chsrobotics.competition2023.commands;

import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.chsrobotics.competition2023.Config;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.lib.drive.differential.DifferentialDriveMode;

public class TeleopDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier linear;
    private final DoubleSupplier rotational;
    private DifferentialDriveMode mode;
    private final DifferentialDriveFeedforward feedforward =
            new DifferentialDriveFeedforward(
                    Constants.COMMAND.TELEOP_DRIVE.FEEDFORWARD_KVLINEAR,
                    Constants.COMMAND.TELEOP_DRIVE.FEEDFORWARD_KALINEAR,
                    Constants.COMMAND.TELEOP_DRIVE.FEEDFORWARD_KVANGULAR,
                    Constants.COMMAND.TELEOP_DRIVE.FEEDFORWARD_KAANGULAR,
                    Constants.COMMAND.TELEOP_DRIVE.FEEDFORWARD_TRACKWIDTH);
    private final DifferentialDriveAccelerationLimiter accelerationLimiter =
            new DifferentialDriveAccelerationLimiter(
                    null,
                    Constants.COMMAND.TELEOP_DRIVE.ACCELERATION_LIMITER_TRACKWIDTH,
                    Constants.COMMAND.TELEOP_DRIVE.ACCELERATION_LIMITER_MAX_LINEAR_ACCEL,
                    Constants.COMMAND.TELEOP_DRIVE.ACCELERATION_LIMITER_MAX_ANGULAR_ACCEL);

    public TeleopDrive(Drivetrain drivetrain, DoubleSupplier linear, DoubleSupplier rotational) {
        this.drivetrain = drivetrain;
        this.linear = linear;
        this.rotational = rotational;
    }

    @Override
    public void execute() {
        mode = Config.TELEOP_DRIVE_MODES.MODE_CHOOSER.getSelected();
        var voltages =
                accelerationLimiter.calculate(
                        drivetrain.getLeftSideVelocity(),
                        drivetrain.getRightSideVelocity(),
                        (mode.execute().right) * 12,
                        (mode.execute().right) * 12);
        drivetrain.setRightVoltages(voltages.left);
        drivetrain.setLeftVoltages(voltages.right);
    }
}
