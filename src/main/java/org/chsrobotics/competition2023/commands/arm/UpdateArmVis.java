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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.telemetry.HighLevelLogger;

public class UpdateArmVis extends CommandBase {
    private final Arm arm;

    private final DoubleSupplier localSetpoint;
    private final DoubleSupplier distalSetpoint;

    private final MechanismLigament2d localActual;
    private final MechanismLigament2d distalActual;

    private final MechanismLigament2d localSetpointVis;
    private final MechanismLigament2d distalSetpointVis;

    public UpdateArmVis(Arm arm, DoubleSupplier localSetpoint, DoubleSupplier distalSetpoint) {
        this.arm = arm;
        this.localSetpoint = localSetpoint;
        this.distalSetpoint = distalSetpoint;

        var vis = new Mechanism2d(6, 3);

        vis.getRoot("drivetrainRoot", 2.5, 0.15)
                .append(
                        new MechanismLigament2d(
                                "drivetrain", 1, 0, 10, new Color8Bit(150, 0, 255)));

        localActual =
                vis.getRoot(
                                "root",
                                3 + Constants.SUBSYSTEM.ARM.ROBOT_TO_ARM.getX(),
                                0.15 + Constants.SUBSYSTEM.ARM.ROBOT_TO_ARM.getZ())
                        .append(
                                new MechanismLigament2d(
                                        "localActual",
                                        Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                                        arm.getLocalAngleRadians()));
        distalActual =
                localActual.append(
                        new MechanismLigament2d(
                                "distalActual",
                                Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS,
                                arm.getDistalAngleRadians()));

        localSetpointVis =
                vis.getRoot("root", 2, 2)
                        .append(
                                new MechanismLigament2d(
                                        "localSetpoint",
                                        Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                                        localSetpoint.getAsDouble(),
                                        6,
                                        new Color8Bit(0, 0, 255)));

        distalSetpointVis =
                localSetpointVis.append(
                        new MechanismLigament2d(
                                "distalSetpoint",
                                Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS,
                                distalSetpoint.getAsDouble(),
                                6,
                                new Color8Bit(0, 0, 255)));

        HighLevelLogger.getInstance().publishSendable("armVis", vis);
    }

    @Override
    public void execute() {
        localActual.setAngle(new Rotation2d(arm.getLocalAngleRadians()));
        distalActual.setAngle(new Rotation2d(arm.getDistalAngleRadians()));

        localSetpointVis.setAngle(new Rotation2d(localSetpoint.getAsDouble()));
        distalSetpointVis.setAngle(new Rotation2d(distalSetpoint.getAsDouble()));
    }
}
