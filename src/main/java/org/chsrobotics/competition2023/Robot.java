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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.chsrobotics.competition2023.commands.ClawCommand;
import org.chsrobotics.competition2023.commands.arm.ArmSetpointControl;
import org.chsrobotics.competition2023.commands.arm.JacobianControl;
import org.chsrobotics.competition2023.commands.arm.SimpleArmTest;
import org.chsrobotics.competition2023.commands.arm.UpdateArmVis;
import org.chsrobotics.competition2023.commands.drivetrain.TeleopDrive;
import org.chsrobotics.competition2023.subsystems.*;
import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.input.JoystickButton;
import org.chsrobotics.lib.input.VirtualJoystickButton;
import org.chsrobotics.lib.input.XboxController;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.util.SRobot;

public class Robot extends SRobot {
    private static final Simulation sim = Simulation.getInstance();

    private static final Localizer localizer = Localizer.getInstance();

    private static final PowerDistributionHub pdh = PowerDistributionHub.getInstance();

    private static final Drivetrain drivetrain = Drivetrain.getInstance();

    private static final Arm arm = Arm.getInstance();

    private static final Claw claw = Claw.getInstance();

    private static final CommandScheduler scheduler = CommandScheduler.getInstance();

    private static long cycleCounter = 0;

    private static final Timer uptimer = new Timer();

    private static final XboxController driverController = new XboxController(0);
    private static final XboxController operatorController = new XboxController(1);
    private static final GenericHID buttonBox = new GenericHID(2);

    private final JoystickAxis driveLin = driverController.leftStickVerticalAxis();
    private final JoystickAxis driveRot = driverController.rightStickHorizontalAxis();
    private final JoystickButton shift =
            new VirtualJoystickButton(driverController.rightTriggerAxis(), 0.1, 1, false);
    private final JoystickAxis slowAxis = driverController.leftTriggerAxis();

    private final JoystickButton brakeButton = driverController.rightBumperButton();

    private final JoystickAxis operatorLeftHorizontal =
            operatorController.leftStickHorizontalAxis();
    private final JoystickAxis operatorLeftVertical = operatorController.leftStickVerticalAxis();
    private final JoystickAxis operatorRightVertical = operatorController.rightStickVerticalAxis();
    private final JoystickAxis operatorLeftTrigger = operatorController.leftTriggerAxis();
    private final JoystickAxis operatorRightTrigger = operatorController.rightTriggerAxis();
    private final JoystickButton jacobianInvert = operatorController.leftBumperButton();
    private final JoystickButton operatorAButton = operatorController.AButton();
    private final JoystickButton operatorBButton = operatorController.BButton();
    private final JoystickButton jacobianSlowMode = operatorController.rightBumperButton();

    private final JoystickButton clawButton = operatorController.XButton();

    private double localSetpoint;
    private double distalSetpoint;
    private double jacobianScaling;

    private final Command jacobianControlCommand =
            new ParallelCommandGroup(
                    new JacobianControl(
                            arm,
                            operatorLeftHorizontal,
                            operatorLeftVertical,
                            jacobianInvert,
                            () -> jacobianScaling),
                    new UpdateArmVis(arm, () -> 0, () -> 0));

    private final Command simpleControlCommand =
            new ParallelCommandGroup(
                    new SimpleArmTest(arm, operatorLeftVertical, operatorRightVertical, 3),
                    new UpdateArmVis(arm, () -> 0, () -> 0));

    private final Command setpointControlCommand =
            new ParallelCommandGroup(
                    new ArmSetpointControl(arm, () -> localSetpoint, () -> distalSetpoint),
                    new UpdateArmVis(arm, () -> localSetpoint, () -> distalSetpoint));

    private void setAllianceLeds() {
        DriverStation.Alliance alliance = DriverStation.getAlliance();
        if (alliance == DriverStation.Alliance.Red) {
            LedMatrix.getInstance().setState(LedMatrix.State.RED);
        } else {
            LedMatrix.getInstance().setState(LedMatrix.State.BLUE);
        }
    }

    @Override
    public void stateTransition(RobotState from, RobotState to) {
        if (to != RobotState.DISABLED) {
            LedMatrix.getInstance().setState(LedMatrix.State.OFF);
        }

        if (from == RobotState.NONE && to == RobotState.DISABLED) {
            // robot initialization
            HighLevelLogger.getInstance().startLogging();
            HighLevelLogger.getInstance().logMessage("*******ROBOT STARTUP*******");
            HighLevelLogger.getInstance().logMessage("997 Competition Robot 2023: Mantis");

            HighLevelLogger.getInstance().autoGenerateLogs("", "system");

            DriverStation.startDataLog(HighLevelLogger.getInstance().getLog(), true);

            Config.publishChoosers();

            driveRot.setInverted(true);
            driveLin.setInverted(true);

            driveRot.addDeadband(0.1);
            driveLin.addDeadband(0.1);

            operatorLeftHorizontal.addDeadband(0.1);
            operatorLeftVertical.addDeadband(0.1);

            operatorLeftVertical.setInverted(true);
            operatorLeftHorizontal.setInverted(true);

            uptimer.reset();
            uptimer.start();

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
            distalSetpoint = Arm.getInstance().getDistalAngleRadians();
            localSetpoint = Arm.getInstance().getLocalAngleRadians();

            scheduler.schedule(new ClawCommand(claw, clawButton));

            scheduler.schedule(
                    new TeleopDrive(
                            drivetrain,
                            driveLin,
                            driveRot,
                            operatorLeftTrigger,
                            operatorRightTrigger,
                            operatorAButton,
                            operatorBButton,
                            shift,
                            brakeButton,
                            slowAxis));

            if (Config.ARM_MODES.ARM_MODE_CHOOSER.getSelected()
                    == Config.ARM_MODES.ARM_MODE.JACOBIAN) {
                scheduler.schedule(jacobianControlCommand);
            } else {
                scheduler.schedule(simpleControlCommand);
            }

        } else if (to == RobotState.AUTONOMOUS) {
            scheduler.schedule(
                    Autos.getAutoCommand(Config.AUTO_MODES.MODE_CHOOSER.getSelected(), drivetrain));
        }
    }

    @Override
    public void periodic(RobotState state) {
        if (state == RobotState.DISABLED) {
            setAllianceLeds();
        }

        if (jacobianSlowMode.getAsBoolean()) {
            jacobianScaling = Constants.COMMAND.ARM_JACOBIAN_CONTROL.INPUT_SCALING_SLOW;
        } else {
            jacobianScaling = Constants.COMMAND.ARM_JACOBIAN_CONTROL.INPUT_SCALING_FULL;
        }

        if (buttonBox.getRawButton(5)) {
            CommandScheduler.getInstance().schedule(setpointControlCommand);
            localSetpoint = Math.PI / 2;
            distalSetpoint = 0;
        } else if (buttonBox.getRawButton(6)) {
            CommandScheduler.getInstance().schedule(setpointControlCommand);
            localSetpoint = 1.45;
            distalSetpoint = -1.5;
        } else if (buttonBox.getRawButton(7)) {
            CommandScheduler.getInstance().schedule(setpointControlCommand);
            localSetpoint = 0.9;
            distalSetpoint = -0.15;
        } else if (buttonBox.getRawButton(8)) {
            CommandScheduler.getInstance().schedule(setpointControlCommand);
            localSetpoint = Math.PI / 2;
            distalSetpoint = -Math.PI / 2;
        } else if (buttonBox.getRawButton(9)) {
            scheduler.schedule(jacobianControlCommand);
        }

        scheduler.run();

        HighLevelLogger.getInstance().updateLogs();

        cycleCounter++;

        localizer.periodic();

        if (!isReal()) {
            sim.periodic();
        }
    }
}
