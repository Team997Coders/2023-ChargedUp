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

import org.chsrobotics.lib.telemetry.DashboardChooser;
import org.chsrobotics.lib.telemetry.HighLevelLogger;

public class Config {
    public static final class AUTO_MODES {
        public static enum MODES implements DashboardChooser.Option {
            NOTHING("Do Nothing"),
            MOBILITY("Mobility"),
            LOW_CUBE("Low Cube"),
            MOBILITY_LOW_CUBE("Low Cube + Mobility"),
            BALANCE("Balance"),
            MOBILITY_BALANCE("Mobility + Balance"),
            BALANCE_LOW_CUBE("Low Cube + Balance"),
            MOBILITY_BALANCE_LOW_CUBE("Low Cube + Mobility + Balance");

            private final String displayName;

            @Override
            public String getDisplayName() {
                return displayName;
            }

            MODES(String displayName) {
                this.displayName = displayName;
            }
        }

        public static final DashboardChooser<MODES> MODE_CHOOSER =
                DashboardChooser.fromEnum(MODES.class, MODES.NOTHING);
    }

    public static final class TELEOP_DRIVE_MODES {
        public static enum LINEAR_MODIFIER implements DashboardChooser.Option {
            FULL(1, "Full"),
            PARTIAL(0.75, "Partial"),
            HALF(0.5, "Half"),
            SLOW(0.25, "Slow");

            public final double value;
            private final String displayName;

            @Override
            public String getDisplayName() {
                return displayName;
            }

            LINEAR_MODIFIER(double value, String displayName) {
                this.value = value;
                this.displayName = displayName;
            }
        }

        public static final DashboardChooser<LINEAR_MODIFIER> LINEAR_MODIFIER_CHOOSER =
                DashboardChooser.fromEnum(LINEAR_MODIFIER.class, LINEAR_MODIFIER.FULL);

        public static enum LINEAR_RAMP_RATE implements DashboardChooser.Option {
            NONE(0, "None"),
            FAST(10, "Fast"),
            MEDIUM(5, "Medium"),
            SLOW(2, "Slow");

            public final double value;
            private final String displayName;

            @Override
            public String getDisplayName() {
                return displayName;
            }

            LINEAR_RAMP_RATE(double value, String displayName) {
                this.value = value;
                this.displayName = displayName;
            }
        }

        public static final DashboardChooser<LINEAR_RAMP_RATE> LINEAR_RAMP_RATE_CHOOSER =
                DashboardChooser.fromEnum(LINEAR_RAMP_RATE.class, LINEAR_RAMP_RATE.NONE);

        public static enum ANGULAR_MODIFIER implements DashboardChooser.Option {
            FULL(1, "Full"),
            PARTIAL(0.75, "Partial"),
            HALF(0.5, "Half"),
            SLOW(0.25, "Slow");

            public final double value;

            private final String displayName;

            @Override
            public String getDisplayName() {
                return displayName;
            }

            ANGULAR_MODIFIER(double value, String displayName) {
                this.value = value;
                this.displayName = displayName;
            }
        }

        public static final DashboardChooser<ANGULAR_MODIFIER> ANGULAR_MODIFIER_CHOOSER =
                DashboardChooser.fromEnum(ANGULAR_MODIFIER.class, ANGULAR_MODIFIER.FULL);

        public static enum ANGULAR_RAMP_RATE implements DashboardChooser.Option {
            NONE(0, "None"),
            FAST(10, "Fast"),
            MEDIUM(5, "Medium"),
            SLOW(2, "Slow");

            public final double value;
            private final String displayName;

            @Override
            public String getDisplayName() {
                return displayName;
            }

            ANGULAR_RAMP_RATE(double value, String displayName) {
                this.value = value;
                this.displayName = displayName;
            }
        }

        public static final DashboardChooser<ANGULAR_RAMP_RATE> ANGULAR_RAMP_RATE_CHOOSER =
                DashboardChooser.fromEnum(ANGULAR_RAMP_RATE.class, ANGULAR_RAMP_RATE.NONE);

        public static enum MODES implements DashboardChooser.Option {
            ARCADE("Arcade"),
            CURVATURE("Curvature"),
            ARCADE_CURVATURE_MIX_EVEN("Arcade-Curvature Mix (Even Bias)"),
            ARCADE_CURVATURE_MIX_BIAS_CURVATURE("Arcade-Curvature Mix (Curvature Bias)"),
            ARCADE_CURVATURE_MIX_BIAS_ARCADE("Arcade-Curvature Mix (Arcade Bias)");

            private final String displayName;

            @Override
            public String getDisplayName() {
                return displayName;
            }

            MODES(String displayName) {
                this.displayName = displayName;
            }
        }

        public static final DashboardChooser<MODES> MODE_CHOOSER =
                DashboardChooser.fromEnum(MODES.class, MODES.ARCADE);
    }

    public static void publishChoosers() {
        HighLevelLogger.getInstance()
                .publishSendable("teleopDriveModeChooser", TELEOP_DRIVE_MODES.MODE_CHOOSER);

        HighLevelLogger.getInstance()
                .publishSendable(
                        "teleopDriveLinearModifierChooser",
                        TELEOP_DRIVE_MODES.LINEAR_MODIFIER_CHOOSER);

        HighLevelLogger.getInstance()
                .publishSendable(
                        "teleopDriveAngularModifierChooser",
                        TELEOP_DRIVE_MODES.ANGULAR_MODIFIER_CHOOSER);

        HighLevelLogger.getInstance()
                .publishSendable(
                        "teleopDriveLinearRampRateChooser",
                        TELEOP_DRIVE_MODES.LINEAR_RAMP_RATE_CHOOSER);

        HighLevelLogger.getInstance()
                .publishSendable(
                        "teleopDriveAngularRampRateChooser",
                        TELEOP_DRIVE_MODES.ANGULAR_RAMP_RATE_CHOOSER);

        HighLevelLogger.getInstance().publishSendable("autoModeChooser", AUTO_MODES.MODE_CHOOSER);
    }
}
