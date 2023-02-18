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

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.input.JoystickAxis;

public class SimpleArmTest extends CommandBase {
    private final Arm arm;

    private final JoystickAxis distalInput;
    private final JoystickAxis localInput;

    private final double scaling;

    public SimpleArmTest(
            Arm arm, JoystickAxis distalInput, JoystickAxis localInput, double scaling) {
        addRequirements(arm);
        this.arm = arm;

        this.distalInput = distalInput;
        this.localInput = localInput;

        this.scaling = scaling;
    }

    @Override
    public void execute() {
        arm.setVoltages(localInput.getValue() * scaling, distalInput.getValue() * scaling);
    }
}
