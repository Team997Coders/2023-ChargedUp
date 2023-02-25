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
package org.chsrobotics.competition2023.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.chsrobotics.competition2023.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;

    private final BooleanSupplier extendedSupplier;

    public IntakeCommand(Intake intake, BooleanSupplier extendedSupplier) {
        this.intake = intake;

        this.extendedSupplier = extendedSupplier;
    }

    @Override
    public void initialize() {
        intake.setVoltage(0);
    }

    @Override
    public void execute() {
        intake.setDeployed(extendedSupplier.getAsBoolean());
        intake.setDeployed(false);
    }
}
