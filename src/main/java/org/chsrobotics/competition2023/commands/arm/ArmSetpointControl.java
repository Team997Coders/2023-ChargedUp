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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.controllers.feedback.PID;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.models.DoubleJointedArmModel;
import org.chsrobotics.lib.telemetry.Logger;

// TODO: new command loggers
public class ArmSetpointControl extends CommandBase {
    private final Arm arm;

    private final DoubleSupplier localAngleLambda;
    private final DoubleSupplier distalAngleLambda;

    private final DoubleSupplier localVelLamdba;
    private final DoubleSupplier distalVelLamdba;

    private final DoubleSupplier localAccelLambda;
    private final DoubleSupplier distalAccelLambda;

    private final DoubleJointedArmModel armSim =
            new DoubleJointedArmModel(
                    Constants.SUBSYSTEM.ARM.LOCAL_MASS_KG,
                    Constants.SUBSYSTEM.ARM.LOCAL_COM_POSITION_FROM_ROOT_METERS,
                    Constants.SUBSYSTEM.ARM.LOCAL_MOMENT_ABOUT_COM,
                    Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                    DCMotor.getNEO(2)
                            .withReduction(
                                    Constants.SUBSYSTEM.ARM.LOCAL_MOTORS_CONVERSION_HELPER
                                            .toDoubleRatioOutputToInput()),
                    Constants.SUBSYSTEM.ARM.DISTAL_MASS_KG,
                    Constants.SUBSYSTEM.ARM.DISTAL_COM_POSITION_FROM_ROOT_METERS,
                    Constants.SUBSYSTEM.ARM.DISTAL_MOMENT_ABOUT_COM,
                    DCMotor.getNEO(1)
                            .withReduction(
                                    Constants.SUBSYSTEM.ARM.DISTAL_MOTOR_CONVERSION_HELPER
                                            .toDoubleRatioOutputToInput()));

    private final String subdirString = "armSetpointControl";

    private final Logger<Double> localFeedforwardLogger =
            new Logger<>("localFeedforward_v", subdirString);

    private final Logger<Double> distalFeedforwardLogger =
            new Logger<>("distalFeedforward_v", subdirString);

    private final Logger<Double> localSetpointLogger =
            new Logger<>("localSetpoint_rad", subdirString);

    private final Logger<Double> distalSetpointLogger =
            new Logger<>("distalSetpoint_rad", subdirString);

    private final PID localController;
    private final PID distalController;

    public ArmSetpointControl(
            Arm arm,
            DoubleSupplier localAngleRadians,
            DoubleSupplier distalAngleRadians,
            DoubleSupplier localAngularVelocityRadPSec,
            DoubleSupplier distalAngularVelocityRadPSec,
            DoubleSupplier localAngularAccelerationRadPSecSquared,
            DoubleSupplier distalAngularAccelerationRadPSecSquared) {
        addRequirements(arm);
        this.arm = arm;

        localAngleLambda = localAngleRadians;
        distalAngleLambda = distalAngleRadians;

        localVelLamdba = localAngularVelocityRadPSec;
        distalVelLamdba = distalAngularVelocityRadPSec;

        localAccelLambda = localAngularAccelerationRadPSecSquared;
        distalAccelLambda = distalAngularAccelerationRadPSecSquared;

        localController =
                new PID(
                        Constants.COMMAND.ARM_SETPOINT.LOCAL_CONTROLLER_CONSTANTS,
                        Constants.COMMAND.ARM_SETPOINT.LOCAL_CONTROLLER_INTEGRATION_WINDOW,
                        localAngleRadians.getAsDouble(),
                        true);

        distalController =
                new PID(
                        Constants.COMMAND.ARM_SETPOINT.DISTAL_CONTROLLER_CONSTANTS,
                        Constants.COMMAND.ARM_SETPOINT.DISTAL_CONTROLLER_INTEGRATION_WINDOW,
                        distalAngleRadians.getAsDouble(),
                        true);
    }

    public ArmSetpointControl(Arm arm, double localAngleRadians, double distalAngleRadians) {
        this(arm, () -> localAngleRadians, () -> distalAngleRadians);
    }

    public ArmSetpointControl(
            Arm arm, DoubleSupplier localAngleRadians, DoubleSupplier distalAngleRadians) {
        this(arm, localAngleRadians, distalAngleRadians, () -> 0, () -> 0, () -> 0, () -> 0);
    }

    @Override
    public void initialize() {
        localController.autoGenerateLogs("localController", subdirString);
        distalController.autoGenerateLogs("distalController", subdirString);
    }

    @Override
    public void execute() {
        localController.updateLogs();
        distalController.updateLogs();

        double localAngle = UtilityMath.normalizeAngleRadians(arm.getLocalAngleRadians());
        double distalAngle = UtilityMath.normalizeAngleRadians(arm.getDistalAngleRadians());

        double localSetpoint = UtilityMath.normalizeAngleRadians(localAngleLambda.getAsDouble());
        double distalSetpoint = UtilityMath.normalizeAngleRadians(distalAngleLambda.getAsDouble());

        localController.setSetpoint(localSetpoint);
        distalController.setSetpoint(distalSetpoint);

        double localFeedbackU = localController.calculate(localAngle);
        double distalFeedbackU = distalController.calculate(distalAngle);

        Vector<N2> feedforwardU =
                armSim.feedforward(
                        VecBuilder.fill(localSetpoint, distalSetpoint),
                        VecBuilder.fill(
                                localVelLamdba.getAsDouble(), distalVelLamdba.getAsDouble()),
                        VecBuilder.fill(
                                localAccelLambda.getAsDouble(), distalAccelLambda.getAsDouble()));

        localFeedforwardLogger.update(feedforwardU.get(0, 0));
        distalFeedforwardLogger.update(feedforwardU.get(1, 0));

        localSetpointLogger.update(localSetpoint);
        distalSetpointLogger.update(distalSetpoint);

        double localU = localFeedbackU + feedforwardU.get(0, 0);

        double distalU = distalFeedbackU + feedforwardU.get(1, 0);

        arm.setVoltages(localU, distalU);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setVoltages(0, 0);
    }
}
