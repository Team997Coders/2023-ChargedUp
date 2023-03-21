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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.chsrobotics.competition2023.Config.AUTO_MODES.MODES;
import org.chsrobotics.competition2023.commands.drivetrain.AutoBalance;
import org.chsrobotics.competition2023.commands.drivetrain.GoOverRamp;
import org.chsrobotics.competition2023.commands.drivetrain.TimedDrive;
import org.chsrobotics.competition2023.subsystems.Drivetrain;

public class Autos {
    public static Command getAutoCommand(MODES mode, Drivetrain drivetrain) {
        switch (mode) {
            case NOTHING:
                return new InstantCommand();
            case MOBILITY:
                return new TimedDrive(drivetrain, -6, 2);
            case LOW_CUBE:
                return new SequentialCommandGroup(
                        new TimedDrive(drivetrain, -8, 0.5), new TimedDrive(drivetrain, 6, 1));
            case MOBILITY_LOW_CUBE:
                return new SequentialCommandGroup(
                        new TimedDrive(drivetrain, -8, 0.5),
                        new TimedDrive(drivetrain, 6, 1),
                        new TimedDrive(drivetrain, -6, 4));
            case BALANCE:
                return new SequentialCommandGroup(
                        new TimedDrive(drivetrain, -4, 3), new AutoBalance(drivetrain));
            case MOBILITY_BALANCE:
                return new SequentialCommandGroup(
                        new TimedDrive(drivetrain, -5, 3),
                        new GoOverRamp(drivetrain),
                        new TimedDrive(drivetrain, -4, 0.5),
                        new WaitCommand(2),
                        new TimedDrive(drivetrain, 5, 2),
                        new AutoBalance(drivetrain));
            case BALANCE_LOW_CUBE:
                return new SequentialCommandGroup(
                        new TimedDrive(drivetrain, -8, 0.5),
                        new TimedDrive(drivetrain, 6, 1),
                        new TimedDrive(drivetrain, -5, 3),
                        new AutoBalance(drivetrain));
            case MOBILITY_BALANCE_LOW_CUBE:
                return new SequentialCommandGroup(
                        new TimedDrive(drivetrain, -8, 0.5),
                        new TimedDrive(drivetrain, 6, 1),
                        new TimedDrive(drivetrain, -5, 3),
                        new GoOverRamp(drivetrain),
                        new TimedDrive(drivetrain, -4, 0.5),
                        new WaitCommand(2),
                        new TimedDrive(drivetrain, 5, 2),
                        new AutoBalance(drivetrain));
            default:
                return new InstantCommand();
        }
    }
}
