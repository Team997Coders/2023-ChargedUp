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

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.input.JoystickButton;
import org.chsrobotics.lib.models.DoubleJointedArmKinematics;

public class CartesianControl extends ParallelCommandGroup {
    private class JoystickPeriodicHandler extends CommandBase {
        final JoystickAxis xAxis;
        final JoystickAxis yAxis;

        double totalDeltaX = 0;
        double totalDeltaY = 0;

        JoystickPeriodicHandler(JoystickAxis xAxis, JoystickAxis yAxis) {
            this.xAxis = xAxis;
            this.yAxis = yAxis;
        }

        @Override
        public void execute() {
            totalDeltaX +=
                    xAxis.getValue()
                            * 0.02
                            * Constants.COMMAND
                                    .ARM_CARTESIAN_CONTROL
                                    .MAX_SETPOINT_VELOCITY_METERS_PER_SECOND;
            totalDeltaY +=
                    yAxis.getValue()
                            * 0.02
                            * Constants.COMMAND
                                    .ARM_CARTESIAN_CONTROL
                                    .MAX_SETPOINT_VELOCITY_METERS_PER_SECOND;
        }
    }

    private final DoubleJointedArmKinematics kinematics =
            new DoubleJointedArmKinematics(
                    Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                    Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS);

    private final double initialX;
    private final double initialY;

    private double lastValidLocalAngle;
    private double lastValidDistalAngle;

    private final JoystickPeriodicHandler joystickHandler;

    private final JoystickButton flipHandedness;

    public CartesianControl(
            Arm arm, JoystickAxis xAxis, JoystickAxis yAxis, JoystickButton flipHandedness) {
        this.flipHandedness = flipHandedness;

        var armConfig =
                kinematics.forwardKinematics(
                        arm.getLocalAngleRadians(), arm.getDistalAngleRadians());

        initialX = armConfig.endEffectorX;
        initialY = armConfig.endEffectorY;

        lastValidLocalAngle = arm.getLocalAngleRadians();
        lastValidDistalAngle = arm.getDistalAngleRadians();

        joystickHandler = new JoystickPeriodicHandler(xAxis, yAxis);

        addCommands(
                new UpdateArmVis(
                        arm, this::getDesiredLocalAngleRadians, this::getDesiredDistalAngleRadians),
                new ArmSetpointControl(
                        arm, this::getDesiredLocalAngleRadians, this::getDesiredDistalAngleRadians),
                joystickHandler);
    }

    private double getDesiredLocalAngleRadians() {
        double totalX = initialX + joystickHandler.totalDeltaX;
        double totalY = initialY + joystickHandler.totalDeltaY;

        var armConfigs = kinematics.inverseKinematics(totalX, totalY);

        if (armConfigs.firstValue() == null) {
        } else if (flipHandedness.getAsBoolean()) {
            lastValidLocalAngle = armConfigs.secondValue().localAngle;
        } else {
            lastValidLocalAngle = armConfigs.firstValue().localAngle;
        }

        return lastValidLocalAngle;
    }

    private double getDesiredDistalAngleRadians() {
        double totalX = initialX + joystickHandler.totalDeltaX;
        double totalY = initialY + joystickHandler.totalDeltaY;

        var armConfigs = kinematics.inverseKinematics(totalX, totalY);

        if (armConfigs.firstValue() == null) {
        } else if (flipHandedness.getAsBoolean()) {
            lastValidDistalAngle = armConfigs.secondValue().distalAngle;
        } else {
            lastValidDistalAngle = armConfigs.firstValue().distalAngle;
        }

        return lastValidDistalAngle;
    }
}
