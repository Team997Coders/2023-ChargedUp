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
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Localizer;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.lib.telemetry.Logger;

public class TrajectoryFollow extends CommandBase {

    private Drivetrain drivetrain;

    private Trajectory trajectory;

    private Timer timer = new Timer();

    private final String subDirString = "TrajectoryFollowing";
    private final Logger<Double> timerLogger = new Logger<>("epoch_s", subDirString);
    private final Logger<Double> chassisspeedLinearVLogger =
            new Logger<>("Chassis_Speed_Linear_V", subDirString);
    private final Logger<Double> chassisspeedAngularVLogger =
            new Logger<>("Chassis_Speed_Angular_V", subDirString);
    private final Logger<Double> wheelSpeedsLeftSetpointVLogger =
            new Logger<>("Wheel_Speeds_Left_V_Setpoint", subDirString);
    private final Logger<Double> wheelSpeedsRightSetpointVLogger =
            new Logger<>("Wheel_Speeds_Right_V_Setpoint", subDirString);
    private RamseteController ramseteController =
            new RamseteController(
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.K_RAMSETE_B,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.K_RAMSETE_ZETA);

    private DifferentialDriveFeedforward feedforward =
            new DifferentialDriveFeedforward(
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.FEED_FORWARD_KV,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.FEED_FORWARD_KA,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.FEED_FORWARD_ANGULAR_KV,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.FEED_FORWARD_ANGULAR_KA);

    public TrajectoryFollow(Drivetrain drivetrain, Trajectory trajectory) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        timerLogger.update(timer.get());
        ChassisSpeeds chassisSpeed =
                ramseteController.calculate(
                        Localizer.getInstance().getEstimatedPose(), trajectory.sample(timer.get()));
        chassisspeedAngularVLogger.update(chassisSpeed.omegaRadiansPerSecond);
        chassisspeedLinearVLogger.update(chassisSpeed.vxMetersPerSecond);
        DifferentialDriveWheelSpeeds wheelSpeeds =
                Constants.COMMAND.TRAJECTORY_FOLLOWING.K_DRIVE_KINEMATICS.toWheelSpeeds(
                        chassisSpeed);
        wheelSpeedsLeftSetpointVLogger.update(wheelSpeeds.leftMetersPerSecond);
        wheelSpeedsRightSetpointVLogger.update(wheelSpeeds.rightMetersPerSecond);
        DifferentialDriveWheelVoltages voltages =
                feedforward.calculate(
                        /*no drivetrain velocity yet */ 0,
                        wheelSpeeds.leftMetersPerSecond, /*still */
                        0,
                        wheelSpeeds.rightMetersPerSecond,
                        0.02);
        drivetrain.setLeftVoltages(voltages.left);
        drivetrain.setRightVoltages(voltages.right);
    }
}
