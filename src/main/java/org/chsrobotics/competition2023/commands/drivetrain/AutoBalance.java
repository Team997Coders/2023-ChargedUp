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
package org.chsrobotics.competition2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.InertialMeasurement;
import org.chsrobotics.lib.controllers.feedback.PID;

public class AutoBalance extends CommandBase {
    private final Drivetrain drivetrain;
    private final PID pid =
            new PID(
                    Constants.COMMAND.AUTO_BALANCE.KP,
                    Constants.COMMAND.AUTO_BALANCE.KI,
                    Constants.COMMAND.AUTO_BALANCE.KD,
                    0);

    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double u = -pid.calculate(InertialMeasurement.getInstance().getPitchRadians());

        drivetrain.setLeftVoltages(u);
        drivetrain.setRightVoltages(u);

        if (pid.atSetpoint()) {
            drivetrain.setBrakeMode(false);
        }
    }
}
