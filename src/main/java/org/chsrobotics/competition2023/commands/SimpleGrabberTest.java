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
import org.chsrobotics.competition2023.subsystems.Grabber;
import org.chsrobotics.lib.input.JoystickButton;

public class SimpleGrabberTest extends CommandBase {
    private final Grabber grabber;
    private final JoystickButton toggle;

    private boolean lastState = false;

    public SimpleGrabberTest(Grabber grabber, JoystickButton toggle) {
        addRequirements(grabber);
        this.grabber = grabber;

        this.toggle = toggle;
    }

    @Override
    public void execute() {
        if (toggle.getAsBoolean() && !lastState) grabber.setSolenoid(!grabber.isClosed());

        lastState = toggle.getAsBoolean();
    }
}
