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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.Grabber;
import org.chsrobotics.competition2023.subsystems.InertialMeasurement;
import org.chsrobotics.competition2023.subsystems.Intake;
import org.chsrobotics.competition2023.subsystems.Pneumatics;
import org.chsrobotics.competition2023.subsystems.PowerDistributionHub;
import org.chsrobotics.competition2023.subsystems.Vision;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.util.SRobot;

public class Robot extends SRobot {
    private static final Simulation sim = Simulation.getInstance();

    private static final Localizer localizer = Localizer.getInstance();

    private static final PowerDistributionHub pdh = PowerDistributionHub.getInstance();

    private static final InertialMeasurement imu = InertialMeasurement.getInstance();

    private static final Drivetrain drivetrain = Drivetrain.getInstance();

    private static final Grabber grabber = Grabber.getInstance();

    private static final Intake intake = Intake.getInstance();

    private static final Pneumatics pneumatics = Pneumatics.getInstance();

    private static final Vision vision = Vision.getInstance();

    private static final CommandScheduler scheduler = CommandScheduler.getInstance();

    private static long cycleCounter = 0;

    private static final Timer uptimer = new Timer();

    @Override
    public void stateTransition(RobotState from, RobotState to) {
        if (from == RobotState.NONE && to == RobotState.DISABLED) {
            // robot initialization

            HighLevelLogger.getInstance().startLogging();
            HighLevelLogger.getInstance().logMessage("*******ROBOT STARTUP*******");
            HighLevelLogger.getInstance().logMessage("997 Competition Robot 2023");

            HighLevelLogger.getInstance().autoGenerateLogs("", "system");

            DriverStation.startDataLog(HighLevelLogger.getInstance().getLog(), true);

            uptimer.reset();
            uptimer.start();

        } else if (to == RobotState.TEST) {
            // test mode
            CommandScheduler.getInstance().cancelAll();
        } else if ((DriverStation.isFMSAttached()
                        && from == RobotState.TELEOPERATED
                        && to == RobotState.DISABLED)
                || (!DriverStation.isFMSAttached() && to == RobotState.DISABLED)) {
            // in FMS mode, there's a disabled period of varying length between auto and teleop, so
            // we can't just use that
            HighLevelLogger.getInstance().logMessage("***** Robot Disabled *****");
            HighLevelLogger.getInstance()
                    .logMessage("Total energy use (Joules): " + pdh.getTotalEnergyUsed());
            HighLevelLogger.getInstance().logMessage("Loop cycles: " + cycleCounter);
            HighLevelLogger.getInstance().logMessage("Uptime (s): " + uptimer.get());
        }
    }

    @Override
    public void periodic(RobotState state) {
        scheduler.run();

        HighLevelLogger.getInstance().updateLogs();

        cycleCounter++;

        localizer.periodic();

        if (!isReal()) {
            sim.periodic();
        }
    }
}
