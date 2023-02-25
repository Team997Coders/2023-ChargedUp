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

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Localizer;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.lib.telemetry.Logger;

public class TrajectoryFollow extends CommandBase {

    private final Drivetrain drivetrain;

    private final Trajectory trajectory;

    private final Timer timer = new Timer();

    private final String subdirString = "TrajectoryFollowing";

    private final Logger<Double> timerLogger = new Logger<>("epoch_s", subdirString);
    private final Logger<Double> chassisspeedLinearVLogger =
            new Logger<>("linearVelocitySetpoint_mps", subdirString);
    private final Logger<Double> chassisspeedAngularVLogger =
            new Logger<>("angularVelocitySetpoint_radps", subdirString);
    private final Logger<Double> wheelSpeedsLeftSetpointVLogger =
            new Logger<>("leftSideVelocitySetpoint_mps", subdirString);
    private final Logger<Double> wheelSpeedsRightSetpointVLogger =
            new Logger<>("rightSideVelocitySetpoint_mps", subdirString);
    private final Logger<Double[]> poseSetpointLogger =
            new Logger<>("poseSetpoint_meters_meters_radians", subdirString);

    private final RamseteController ramseteController =
            new RamseteController(
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.K_RAMSETE_B,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.K_RAMSETE_ZETA);

    private final DifferentialDriveFeedforward fastFeedforward =
            new DifferentialDriveFeedforward(
                    Constants.SUBSYSTEM.DRIVETRAIN.FAST_KV_LINEAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.FAST_KA_LINEAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.FAST_KV_ANGULAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.FAST_KA_ANGULAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.TRACKWIDTH_METERS);

    private final DifferentialDriveFeedforward slowFeedforward =
            new DifferentialDriveFeedforward(
                    Constants.SUBSYSTEM.DRIVETRAIN.SLOW_KV_LINEAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.SLOW_KA_LINEAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.SLOW_KV_ANGULAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.SLOW_KA_ANGULAR,
                    Constants.SUBSYSTEM.DRIVETRAIN.TRACKWIDTH_METERS);

    private final boolean setPoseToInitial;

    public TrajectoryFollow(
            Drivetrain drivetrain, Trajectory trajectory, boolean setPoseToInitial) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        this.setPoseToInitial = setPoseToInitial;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (setPoseToInitial) Localizer.getInstance().setPose(trajectory.getInitialPose());

        drivetrain.setShifters(false);
    }

    @Override
    public void execute() {
        var setpoint = trajectory.sample(timer.get());

        poseSetpointLogger.update(
                new Double[] {
                    setpoint.poseMeters.getX(),
                    setpoint.poseMeters.getY(),
                    setpoint.poseMeters.getRotation().getRadians()
                });

        timerLogger.update(timer.get());

        ChassisSpeeds chassisSpeed =
                ramseteController.calculate(Localizer.getInstance().getEstimatedPose(), setpoint);

        chassisspeedAngularVLogger.update(chassisSpeed.omegaRadiansPerSecond);
        chassisspeedLinearVLogger.update(chassisSpeed.vxMetersPerSecond);

        DifferentialDriveWheelSpeeds wheelSpeeds =
                new DifferentialDriveKinematics(Constants.SUBSYSTEM.DRIVETRAIN.TRACKWIDTH_METERS)
                        .toWheelSpeeds(chassisSpeed);

        wheelSpeedsLeftSetpointVLogger.update(wheelSpeeds.leftMetersPerSecond);
        wheelSpeedsRightSetpointVLogger.update(wheelSpeeds.rightMetersPerSecond);

        var feedforward = drivetrain.getShifterSlow() ? slowFeedforward : fastFeedforward;

        var voltages =
                feedforward.calculate(
                        drivetrain.getLeftSideVelocity(),
                        wheelSpeeds.leftMetersPerSecond,
                        drivetrain.getRightSideVelocity(),
                        wheelSpeeds.rightMetersPerSecond,
                        0.02);

        drivetrain.setLeftVoltages(voltages.left);
        drivetrain.setRightVoltages(voltages.right);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setLeftVoltages(0);
        drivetrain.setRightVoltages(0);
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > trajectory.getTotalTimeSeconds());
    }
}
