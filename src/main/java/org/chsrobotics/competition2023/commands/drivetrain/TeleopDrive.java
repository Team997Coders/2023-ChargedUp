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

import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.Map;
import org.chsrobotics.competition2023.Config;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.lib.drive.differential.ArcadeDrive;
import org.chsrobotics.lib.drive.differential.CurvatureDrive;
import org.chsrobotics.lib.drive.differential.DifferentialDriveMode;
import org.chsrobotics.lib.drive.differential.MixedDrive;
import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.input.JoystickButton;
import org.chsrobotics.lib.telemetry.Logger;

public class TeleopDrive extends CommandBase {
    private final Drivetrain drivetrain;

    private final DifferentialDriveAccelerationLimiter fastAccelerationLimiter =
            new DifferentialDriveAccelerationLimiter(
                    Constants.SUBSYSTEM.DRIVETRAIN.FAST_DRIVETRAIN_PLANT,
                    Constants.SUBSYSTEM.DRIVETRAIN.TRACKWIDTH_METERS,
                    Constants.COMMAND.TELEOP_DRIVE.FAST_MAX_LINEAR_ACCEL_M_P_SEC_SQUARED,
                    Constants.COMMAND.TELEOP_DRIVE.FAST_MAX_ANGULAR_ACCEL_RADS_P_SEC_SQUARED);

    private final DifferentialDriveAccelerationLimiter slowAccelerationLimiter =
            new DifferentialDriveAccelerationLimiter(
                    Constants.SUBSYSTEM.DRIVETRAIN.SLOW_DRIVETRAIN_PLANT,
                    Constants.SUBSYSTEM.DRIVETRAIN.TRACKWIDTH_METERS,
                    Constants.COMMAND.TELEOP_DRIVE.SLOW_MAX_LINEAR_ACCEL_M_P_SEC_SQUARED,
                    Constants.COMMAND.TELEOP_DRIVE.SLOW_MAX_ANGULAR_ACCEL_RAD_P_SEC_SQUARED);

    private final JoystickAxis axisA;
    private final JoystickAxis axisB;

    private final JoystickButton shiftButton;

    private final JoystickAxis slowAxis;

    private final JoystickButton brakeButton;

    public TeleopDrive(
            Drivetrain drivetrain,
            JoystickAxis axisA,
            JoystickAxis axisB,
            JoystickButton shiftButton,
            JoystickButton brakeButton,
            JoystickAxis slowAxis) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        this.axisA = axisA;
        this.axisB = axisB;

        this.shiftButton = shiftButton;

        this.brakeButton = brakeButton;

        this.slowAxis = slowAxis;
    }

    private final Logger<Boolean> brakeLogger = new Logger<>("isCoastMode", "drivetrain");

    @Override
    public void execute() {
        drivetrain.setCoastMode(!brakeButton.getAsBoolean());
        brakeLogger.update(drivetrain.getIsCoastMode());

        drivetrain.setShifters(!shiftButton.getAsBoolean());

        DifferentialDriveMode mode;

        double linMod = Config.TELEOP_DRIVE_MODES.LINEAR_MODIFIER_CHOOSER.getSelected().value;
        double angMod = Config.TELEOP_DRIVE_MODES.ANGULAR_MODIFIER_CHOOSER.getSelected().value;

        double linLimit = Config.TELEOP_DRIVE_MODES.LINEAR_RAMP_RATE_CHOOSER.getSelected().value;
        double angLimit = Config.TELEOP_DRIVE_MODES.ANGULAR_RAMP_RATE_CHOOSER.getSelected().value;

        var arcade = new ArcadeDrive(axisA, axisB, linMod, angMod, linLimit, angLimit);
        var curvature = new CurvatureDrive(axisA, axisB, linMod, angMod, linLimit, angLimit, false);

        switch (Config.TELEOP_DRIVE_MODES.MODE_CHOOSER.getSelected()) {
            case ARCADE:
                mode = arcade;
            case CURVATURE:
                mode = curvature;
            case ARCADE_CURVATURE_MIX_EVEN:
                mode = new MixedDrive(Map.of(arcade, 0.5, curvature, 0.5));
            case ARCADE_CURVATURE_MIX_BIAS_ARCADE:
                mode = new MixedDrive(Map.of(arcade, 0.75, curvature, 0.25));
            case ARCADE_CURVATURE_MIX_BIAS_CURVATURE:
                mode = new MixedDrive(Map.of(arcade, 0.25, curvature, 0.75));
            default:
                mode = arcade;
        }

        var limiter =
                drivetrain.getShifterSlow() ? slowAccelerationLimiter : fastAccelerationLimiter;

        var voltages =
                limiter.calculate(
                        drivetrain.getLeftSideVelocity(),
                        drivetrain.getRightSideVelocity(),
                        (mode.execute().left) * Constants.GLOBAL.GLOBAL_NOMINAL_VOLTAGE_VOLTS,
                        (mode.execute().right) * Constants.GLOBAL.GLOBAL_NOMINAL_VOLTAGE_VOLTS);

        // these are commented out, as we literally didn't have time to get the drivetrain
        // feedforward ready

        // drivetrain.setRightVoltages(voltages.right);
        // drivetrain.setLeftVoltages(voltages.left);

        double slowModifier =
                ((1 - slowAxis.getValue()) * (1 - Constants.COMMAND.TELEOP_DRIVE.MIN_SPEED))
                        + Constants.COMMAND.TELEOP_DRIVE.MIN_SPEED;

        drivetrain.setRightVoltages(mode.execute().right * 12 * slowModifier);
        drivetrain.setLeftVoltages(mode.execute().left * 12 * slowModifier);
    }
}
