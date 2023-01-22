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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Localizer;
import org.chsrobotics.competition2023.subsystems.Drivetrain;

public class TrajectoryFollow extends CommandBase {

    DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            Constants.COMMAND.TRAJECTORY_FOLLOWING.KS_VOLTS,
                            Constants.COMMAND.TRAJECTORY_FOLLOWING.KV_VOLTS_SECONDS_PER_METER,
                            Constants.COMMAND
                                    .TRAJECTORY_FOLLOWING
                                    .KA_VOLTS_SECONDSSQUARED_PER_METER),
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.K_DRIVE_KINEMATICS,
                    10);
    TrajectoryConfig config =
            new TrajectoryConfig(
                            Constants.COMMAND.TRAJECTORY_FOLLOWING.K_MAX_SPEED_METERS_PER_SECOND,
                            Constants.COMMAND
                                    .TRAJECTORY_FOLLOWING
                                    .K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.COMMAND.TRAJECTORY_FOLLOWING.K_DRIVE_KINEMATICS)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);
    private Drivetrain drivetrain;
    private Trajectory trajectory;
    private Timer timer = new Timer();
    private RamseteController ramseteController =
            new RamseteController(
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.K_RAMSETE_B,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.K_RAMSETE_ZETA);
    private SimpleMotorFeedforward simpleFeedForward =
            new SimpleMotorFeedforward(
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.KS_VOLTS,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.KV_VOLTS_SECONDS_PER_METER,
                    Constants.COMMAND.TRAJECTORY_FOLLOWING.KA_VOLTS_SECONDSSQUARED_PER_METER);
    private PIDController PID =
            new PIDController(Constants.COMMAND.TRAJECTORY_FOLLOWING.KP_DRIVE_VEL, 0, 0);

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
        var chassisSpeed =
                ramseteController.calculate(
                        Localizer.getInstance().getEstimatedPose(), trajectory.sample(timer.get()));
        var wheelSpeeds =
                Constants.COMMAND.TRAJECTORY_FOLLOWING.K_DRIVE_KINEMATICS.toWheelSpeeds(
                        chassisSpeed);
        var voltages = simpleFeedForward.calculate(0);
        drivetrain.setLeftVoltages(PID.calculate(voltages));
        drivetrain.setRightVoltages(PID.calculate(voltages));
    }
}
