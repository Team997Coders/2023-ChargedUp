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
package org.chsrobotics.competition2023;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.io.File;
import org.chsrobotics.competition2023.commands.SimpleGrabberTest;
import org.chsrobotics.competition2023.commands.TeleopDrive;
import org.chsrobotics.competition2023.commands.TrajectoryFollow;
import org.chsrobotics.competition2023.commands.arm.ArmSetpointControl;
import org.chsrobotics.competition2023.commands.arm.UpdateArmVis;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.Grabber;
import org.chsrobotics.competition2023.subsystems.PowerDistributionHub;
import org.chsrobotics.competition2023.util.CSpacePackageLoader;
import org.chsrobotics.competition2023.util.CSpacePackageLoader.CSpacePackage;
import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.input.JoystickButton;
import org.chsrobotics.lib.input.VirtualJoystickButton;
import org.chsrobotics.lib.input.XboxController;
import org.chsrobotics.lib.math.geometry.Polygon;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.trajectory.planning.ConfigurationSpace;
import org.chsrobotics.lib.trajectory.planning.ConfigurationSpace.ConfigurationSpaceDimension;
import org.chsrobotics.lib.util.NodeGraph;
import org.chsrobotics.lib.util.SRobot;

public class Robot extends SRobot {
    private static final Simulation sim = Simulation.getInstance();

    private static final Localizer localizer = Localizer.getInstance();

    private static final PowerDistributionHub pdh = PowerDistributionHub.getInstance();

    private static final Drivetrain drivetrain = Drivetrain.getInstance();

    private static final CommandScheduler scheduler = CommandScheduler.getInstance();

    private static long cycleCounter = 0;

    private static final Timer uptimer = new Timer();

    private static final XboxController driverController = new XboxController(0);
    private static final XboxController operatorController = new XboxController(1);

    private final Trajectory trajectory =
            PathPlanner.loadPath("testTraj", new PathConstraints(3, 4));

    private final TrajectoryFollow trajectoryFollow =
            new TrajectoryFollow(drivetrain, trajectory, true);

    private final JoystickAxis driveLin = driverController.leftStickVerticalAxis();
    private final JoystickAxis driveRot = driverController.rightStickHorizontalAxis();
    private final JoystickButton shift =
            new VirtualJoystickButton(driverController.rightTriggerAxis(), 0.1, 1, false);
    private final JoystickButton brake = driverController.BButton();

    private final JoystickAxis localVoltage = operatorController.rightStickHorizontalAxis();
    private final JoystickAxis distalVoltage = operatorController.leftStickHorizontalAxis();

    private final JoystickButton grabberButton = operatorController.XButton();

    @Override
    public void stateTransition(RobotState from, RobotState to) {
        if (from == RobotState.NONE && to == RobotState.DISABLED) {
            // robot initialization
            HighLevelLogger.getInstance().startLogging();
            HighLevelLogger.getInstance().logMessage("*******ROBOT STARTUP*******");
            HighLevelLogger.getInstance().logMessage("997 Competition Robot 2023: Mantis");

            HighLevelLogger.getInstance().autoGenerateLogs("", "system");

            DriverStation.startDataLog(HighLevelLogger.getInstance().getLog(), true);

            Config.publishChoosers();

            driveRot.setInverted(true);
            driveLin.setInverted(false);

            driveRot.addDeadband(0.05);
            driveLin.addDeadband(0.05);

            localVoltage.addDeadband(0.1);
            distalVoltage.addDeadband(0.1);

            uptimer.reset();
            uptimer.start();

            NodeGraph<Vector2D> nodes = new NodeGraph<>();

            CSpacePackage pack =
                    new CSpacePackage(
                            new ConfigurationSpace(
                                    new ConfigurationSpaceDimension(
                                            -Math.PI / 2, (3 * Math.PI) / 2, false),
                                    new ConfigurationSpaceDimension(0, 2 * Math.PI, false),
                                    new Polygon(new Vector2D(cycleCounter, cycleCounter))),
                            nodes);

            CSpacePackageLoader.writePackage(
                    new File(Filesystem.getDeployDirectory(), "cspace/feb27.json"), pack);

            // have to schedule a dummy command to get the arm telemetry to show up in NT for
            // whatever reason-- this doesn't happen with other subsystems
            scheduler.schedule(
                    new InstantCommand(
                            new Runnable() {
                                @Override
                                public void run() {}
                            },
                            Arm.getInstance()));

        } else if (to == RobotState.TEST) {
            // test mode
            CommandScheduler.getInstance().cancelAll();
        } else if ((DriverStation.isFMSAttached()
                        && from == RobotState.TELEOPERATED
                        && to == RobotState.DISABLED)
                || (!DriverStation.isFMSAttached() && to == RobotState.DISABLED)) {
            // in FMS mode, there's a disabled period of varying length between auto and teleop, so
            // we can't just use that
            HighLevelLogger.getInstance().logMessage("***** Robot Disabled *****");
            HighLevelLogger.getInstance()
                    .logMessage("Total energy use (Joules): " + pdh.getTotalEnergyUsed());
            HighLevelLogger.getInstance().logMessage("Loop cycles: " + cycleCounter);
            HighLevelLogger.getInstance().logMessage("Uptime (s): " + uptimer.get());
        } else if (to == RobotState.TELEOPERATED) {
            scheduler.schedule(new TeleopDrive(drivetrain, driveLin, driveRot, shift, brake));

            scheduler.schedule(new SimpleGrabberTest(Grabber.getInstance(), grabberButton));

            scheduler.schedule(new TeleopDrive(drivetrain, driveLin, driveRot, shift, brake));

            // scheduler.schedule(
            //        new ArmNavigate(
            //                Arm.getInstance(),
            //                Constants.COMMAND.ARM_NAVIGATE.FREE,
            //                1,
            //                1));

            var axisH = operatorController.rightStickHorizontalAxis();

            var axisV = operatorController.rightStickVerticalAxis();

            axisH.addDeadband(0.05);

            axisV.setInverted(true);
            axisV.addDeadband(0.05);

            // scheduler.schedule(
            //        new CartesianControl(
            //                Arm.getInstance(), axisH, axisV, operatorController.AButton()));

            scheduler.schedule(
                    new ArmSetpointControl(Arm.getInstance(), 1, 1),
                    new UpdateArmVis(Arm.getInstance(), () -> 1, () -> 1));

        } else if (to == RobotState.AUTONOMOUS) {
            scheduler.schedule(trajectoryFollow);
        }
    }

    @Override
    public void periodic(RobotState state) {
        scheduler.run();

        HighLevelLogger.getInstance().updateLogs();

        cycleCounter++;

        localizer.periodic();

        if (!isReal()) {
            sim.periodic();
        }
    }
}
