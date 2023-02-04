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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.controllers.feedback.PID;
import org.chsrobotics.lib.controllers.feedforward.TwoJointArmFeedforward;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.util.Tuple2;

// TODO: new command loggers
public class ArmSetpointControl extends CommandBase {
    private final Arm arm;

    private final DoubleSupplier localAngleRadians;
    private final DoubleSupplier distalAngleRadians;

    private final TwoJointArmFeedforward feedforward =
            new TwoJointArmFeedforward(
                    Constants.SUBSYSTEM.ARM.LOCAL_COM_POSITION_FROM_ROOT_METERS,
                    Constants.SUBSYSTEM.ARM.LOCAL_MASS_KG,
                    Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                    Constants.SUBSYSTEM.ARM.LOCAL_MOMENT_ABOUT_COM,
                    Constants.SUBSYSTEM.ARM.DISTAL_COM_POSITION_FROM_ROOT_METERS,
                    Constants.SUBSYSTEM.ARM.DISTAL_MASS_KG,
                    Constants.SUBSYSTEM.ARM.DISTAL_MOMENT_ABOUT_COM);

    private final PID localController;
    private final PID distalController;

    private final DCMotor localGearbox =
            DCMotor.getNEO(2)
                    .withReduction(
                            Constants.SUBSYSTEM.ARM.LOCAL_NEO_TO_ARM_HELPER
                                    .toDoubleRatioOutputToInput());

    private final DCMotor distalGearbox =
            DCMotor.getNEO(1)
                    .withReduction(
                            Constants.SUBSYSTEM.ARM.DISTAL_NEO_TO_ARM_HELPER
                                    .toDoubleRatioOutputToInput());

    private final String subdirString = "armSetpointControl";

    public ArmSetpointControl(
            Arm arm, DoubleSupplier localAngleRadians, DoubleSupplier distalAngleRadians) {
        addRequirements(arm);
        this.arm = arm;
        this.localAngleRadians = localAngleRadians;
        this.distalAngleRadians = distalAngleRadians;

        localController =
                new PID(
                        Constants.COMMAND.ARM_SETPOINT.LOCAL_CONTROLLER_CONSTANTS,
                        Constants.COMMAND.ARM_SETPOINT.LOCAL_CONTROLLER_INTEGRATION_WINDOW,
                        localAngleRadians.getAsDouble());

        distalController =
                new PID(
                        Constants.COMMAND.ARM_SETPOINT.DISTAL_CONTROLLER_CONSTANTS,
                        Constants.COMMAND.ARM_SETPOINT.DISTAL_CONTROLLER_INTEGRATION_WINDOW,
                        distalAngleRadians.getAsDouble());
    }

    public ArmSetpointControl(Arm arm, double localAngleRadians, double distalAngleRadians) {
        this(arm, () -> localAngleRadians, () -> distalAngleRadians);
    }

    @Override
    public void initialize() {
        localController.autoGenerateLogs("localController", subdirString);
        distalController.autoGenerateLogs("distalController", subdirString);

        arm.setLocalIdleMode(false);
        arm.setDistalIdleMode(false);
    }

    @Override
    public void execute() {
        localController.updateLogs();
        distalController.updateLogs();

        double localWrappedSetpoint =
                UtilityMath.smallestAngleRadiansBetween(
                                localAngleRadians.getAsDouble(),
                                arm.getLocalPotentiometerAngleRadians())
                        + arm.getLocalPotentiometerAngleRadians();

        double distalWrappedSetpoint =
                UtilityMath.smallestAngleRadiansBetween(
                                distalAngleRadians.getAsDouble(),
                                arm.getDistalPotentiometerAngleRadians())
                        + arm.getDistalPotentiometerAngleRadians();

        double localFeedbackU = localController.calculate(localWrappedSetpoint);
        double distalFeedbackU = distalController.calculate(distalWrappedSetpoint);

        var torques =
                feedforward.getFeedforwardTorques(
                        Tuple2.of(
                                localAngleRadians.getAsDouble(), distalAngleRadians.getAsDouble()),
                        Tuple2.of(
                                arm.getLocalVelociyRadiansPerSecond(),
                                arm.getDistalVelocityRadiansPerSecond()));

        double localFeedfowardU =
                (torques.firstValue() * localGearbox.nominalVoltageVolts)
                        / localGearbox.stallTorqueNewtonMeters;

        double distalFeedforwardU =
                (torques.secondValue() * distalGearbox.nominalVoltageVolts)
                        / distalGearbox.stallTorqueNewtonMeters;

        arm.setLocalVoltage(localFeedbackU + localFeedfowardU);
        arm.setDistalVoltage(distalFeedbackU + distalFeedforwardU);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setLocalVoltage(0);
        arm.setDistalVoltage(0);
    }
}
