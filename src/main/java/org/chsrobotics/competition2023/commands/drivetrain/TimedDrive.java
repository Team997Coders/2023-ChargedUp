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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Drivetrain;

public class TimedDrive extends CommandBase {
    private final Timer timer = new Timer();

    private final Drivetrain drivetrain;

    private final double u;

    private final double t;

    public TimedDrive(Drivetrain drivetrain, double u, double t) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;

        this.u = u;
        this.t = t;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        drivetrain.setLeftVoltages(u * Constants.SUBSYSTEM.DRIVETRAIN.LEFT_MOTOR_MULTIPLIER);
        drivetrain.setRightVoltages(u * Constants.SUBSYSTEM.DRIVETRAIN.RIGHT_MOTOR_MULTIPLIER);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setLeftVoltages(0);
        drivetrain.setRightVoltages(0);
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > t);
    }
}
