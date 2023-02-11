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
package org.chsrobotics.competition2023.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Robot;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.telemetry.Logger;

public class PowerDistributionHub implements Subsystem {
    private static final PowerDistributionHub instance = new PowerDistributionHub();

    private final PowerDistribution pdh = new PowerDistribution();

    private double simTotalCurrent = 0;

    private double simTotalEnergy = 0;

    private final String subdirString = "powerDistribution";

    private final Logger<Double> totalCurrentLogger = new Logger<>("totalCurrent_A", subdirString);

    private final Logger<Double> busVoltageLogger = new Logger<>("busVoltage_V", subdirString);

    private final Logger<Double> temperatureLogger = new Logger<>("temperature_C", subdirString);

    private PowerDistributionHub() {
        register();

        // explicitly set as on because all our coprocessors are off that branch
        pdh.setSwitchableChannel(true);
    }

    public static PowerDistributionHub getInstance() {
        return instance;
    }

    public double getCurrent(int channel) {
        return pdh.getCurrent(channel);
    }

    public double getTotalCurrent() {
        if (Robot.isReal()) return pdh.getTotalCurrent();
        else return simTotalCurrent;
    }

    public double getTotalEnergyUsed() {
        if (Robot.isReal()) return pdh.getTotalEnergy();
        else return simTotalEnergy;
    }

    public void setSimState(double totalCurrentDraw) {
        if (!Robot.isReal()) {
            simTotalCurrent = totalCurrentDraw;
        } else {
            HighLevelLogger.getInstance()
                    .logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.getInstance()
                    .logWarning("There might be sim code still running somewhere!");
        }
    }

    @Override
    public void periodic() {
        simTotalEnergy += (simTotalCurrent * TimedRobot.kDefaultPeriod);

        totalCurrentLogger.update(pdh.getTotalCurrent());

        busVoltageLogger.update(pdh.getVoltage());

        temperatureLogger.update(pdh.getTemperature());
    }
}
