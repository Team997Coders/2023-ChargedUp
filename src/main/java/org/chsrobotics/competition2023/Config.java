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

import org.chsrobotics.lib.drive.differential.DifferentialDriveMode;
import org.chsrobotics.lib.telemetry.DashboardChooser;

public class Config {
    public static final class TELEOP_DRIVE_MODES {
        public static enum MODES implements DashboardChooser.Option {
            MODEA,
            MODEB;
        }

        public static final DashboardChooser<DifferentialDriveMode> MODE_CHOOSER =
                DashboardChooser.fromEnum(MODES.class, MODES.MODEA);
    }
}
