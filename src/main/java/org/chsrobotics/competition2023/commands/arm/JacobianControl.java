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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.input.JoystickAxis;

public class JacobianControl extends CommandBase {
    private final JoystickAxis xAxis;
    private final JoystickAxis yAxis;

    private final Arm arm;

    private final DoubleSupplier scalingLambda;

    public JacobianControl(
            Arm arm, JoystickAxis xAxis, JoystickAxis yAxis, DoubleSupplier scalingLambda) {
        addRequirements(arm);

        this.xAxis = xAxis;
        this.yAxis = yAxis;

        this.arm = arm;

        this.scalingLambda = scalingLambda;
    }

    @Override
    public void execute() {
        Matrix<N1, N2> inputMat = new Matrix<>(Nat.N1(), Nat.N2());

        inputMat.set(0, 0, xAxis.getValue() * scalingLambda.getAsDouble());

        inputMat.set(0, 1, yAxis.getValue() * scalingLambda.getAsDouble());

        double lenLocal = Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS;
        double lenDistal = Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS;

        double angLocal = arm.getLocalAngleRadians();
        double angDistal = arm.getDistalAngleRadians();

        Matrix<N2, N2> jacobian = new Matrix<>(Nat.N2(), Nat.N2());

        jacobian.set(
                0,
                0,
                -(lenLocal * Math.sin(angLocal)) - (lenDistal * Math.sin(angLocal + angDistal)));

        jacobian.set(1, 0, -lenDistal * Math.sin(angLocal + angDistal));

        jacobian.set(
                0,
                1,
                (lenLocal * Math.cos(angLocal)) + (lenDistal * Math.cos(angLocal + angDistal)));

        jacobian.set(1, 1, lenDistal * Math.cos(angLocal + angDistal));

        Matrix<N1, N2> outputMat = inputMat.times(jacobian.inv());

        var feedforwardU =
                Constants.SUBSYSTEM.ARM.ARM_MODEL.feedforward(
                        VecBuilder.fill(arm.getLocalAngleRadians(), arm.getDistalAngleRadians()));

        double localU = outputMat.get(0, 0) + feedforwardU.get(0, 0);
        double distalU = outputMat.get(0, 1) + feedforwardU.get(1, 0);

        arm.setVoltages(localU, distalU);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setVoltages(0, 0);
    }
}
