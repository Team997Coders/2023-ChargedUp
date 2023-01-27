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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.InertialMeasurement;
import org.chsrobotics.competition2023.subsystems.PowerDistributionHub;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;

public class Simulation {
    private static final Simulation instance = new Simulation();

    private final DifferentialDrivetrainSim drivetrainSim =
            new DifferentialDrivetrainSim(
                    Constants.SUBSYSTEM.DRIVETRAIN.DRIVETRAIN_PLANT,
                    DCMotor.getNEO(2),
                    1,
                    1,
                    1,
                    null);

    private final DifferentiatingFilter drivetrainAccelerationFilter = new DifferentiatingFilter();

    private Simulation() {}

    public static Simulation getInstance() {
        return instance;
    }

    public void setDrivetrainInputs(double leftVolts, double rightVolts) {
        drivetrainSim.setInputs(leftVolts, rightVolts);
    }

    public void periodic() {
        double totalCurrentDraw = 0;

        drivetrainSim.update(TimedRobot.kDefaultPeriod);

        totalCurrentDraw += drivetrainSim.getCurrentDrawAmps();

        Drivetrain.getInstance()
                .setSimState(
                        drivetrainSim.getLeftPositionMeters(),
                        drivetrainSim.getRightPositionMeters());

        InertialMeasurement.getInstance()
                .setSimState(
                        0,
                        drivetrainSim.getHeading().getRadians(),
                        0,
                        drivetrainAccelerationFilter.calculate(
                                (drivetrainSim.getRightVelocityMetersPerSecond()
                                                + drivetrainSim.getLeftVelocityMetersPerSecond())
                                        / 2,
                                TimedRobot.kDefaultPeriod),
                        0,
                        0);

        PowerDistributionHub.getInstance().setSimState(totalCurrentDraw);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDraw));
    }
}
