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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.InertialMeasurement;
import org.chsrobotics.competition2023.subsystems.PowerDistributionHub;
import org.chsrobotics.competition2023.subsystems.Vision;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.models.DoubleJointedArmModel;

public class Simulation {
    private static final Simulation instance = new Simulation();

    private final DifferentialDrivetrainSim fastDrivetrainSim =
            new DifferentialDrivetrainSim(
                    Constants.SUBSYSTEM.DRIVETRAIN.FAST_DRIVETRAIN_PLANT,
                    DCMotor.getNEO(2),
                    Constants.SUBSYSTEM.DRIVETRAIN.FAST_GEAR_RATIO.toDoubleRatioOutputToInput(),
                    Constants.SUBSYSTEM.DRIVETRAIN.TRACKWIDTH_METERS,
                    Constants.SUBSYSTEM.DRIVETRAIN.WHEEL_RADIUS_METERS,
                    null);

    private final DifferentialDrivetrainSim slowDrivetrainSim =
            new DifferentialDrivetrainSim(
                    Constants.SUBSYSTEM.DRIVETRAIN.SLOW_DRIVETRAIN_PLANT,
                    DCMotor.getNEO(2),
                    Constants.SUBSYSTEM.DRIVETRAIN.SLOW_GEAR_RATIO.toDoubleRatioOutputToInput(),
                    Constants.SUBSYSTEM.DRIVETRAIN.TRACKWIDTH_METERS,
                    Constants.SUBSYSTEM.DRIVETRAIN.WHEEL_RADIUS_METERS,
                    null);

    private final DoubleJointedArmModel armSim =
            new DoubleJointedArmModel(
                    Constants.SUBSYSTEM.ARM.LOCAL_MASS_KG,
                    Constants.SUBSYSTEM.ARM.LOCAL_COM_POSITION_FROM_ROOT_METERS,
                    Constants.SUBSYSTEM.ARM.LOCAL_MOMENT_ABOUT_COM,
                    Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                    DCMotor.getNEO(2)
                            .withReduction(
                                    Constants.SUBSYSTEM.ARM.LOCAL_MOTORS_CONVERSION_HELPER
                                            .toDoubleRatioOutputToInput()),
                    Constants.SUBSYSTEM.ARM.DISTAL_MASS_KG,
                    Constants.SUBSYSTEM.ARM.DISTAL_COM_POSITION_FROM_ROOT_METERS,
                    Constants.SUBSYSTEM.ARM.DISTAL_MOMENT_ABOUT_COM,
                    DCMotor.getNEO(1)
                            .withReduction(
                                    Constants.SUBSYSTEM.ARM.DISTAL_MOTOR_CONVERSION_HELPER
                                            .toDoubleRatioOutputToInput()));

    private Vector<N2> armInputVoltages = VecBuilder.fill(0, 0);

    private Matrix<N2, N2> armState = new Matrix<>(N2.instance, N2.instance);

    private final DifferentiatingFilter drivetrainAccelerationFilter = new DifferentiatingFilter();

    private Simulation() {}

    public static Simulation getInstance() {
        return instance;
    }

    public void setDrivetrainInputs(double leftVolts, double rightVolts) {
        fastDrivetrainSim.setInputs(leftVolts, rightVolts);
        slowDrivetrainSim.setInputs(leftVolts, rightVolts);
    }

    public void setArmInputs(double localVoltage, double distalVoltage) {
        armInputVoltages = VecBuilder.fill(localVoltage, distalVoltage);
    }

    public void periodic() {
        double totalCurrentDraw = 0;

        DifferentialDrivetrainSim currentSim =
                Drivetrain.getInstance().getShifterSlow() ? slowDrivetrainSim : fastDrivetrainSim;

        armState = armSim.simulate(armState, armInputVoltages, 0.02);

        totalCurrentDraw += currentSim.getCurrentDrawAmps();

        Vision.getInstance().setSimState(new Pose3d(currentSim.getPose()));

        Drivetrain.getInstance()
                .setSimState(
                        currentSim.getLeftPositionMeters(), currentSim.getRightPositionMeters());

        InertialMeasurement.getInstance()
                .setSimState(
                        0,
                        currentSim.getHeading().getRadians(),
                        0,
                        drivetrainAccelerationFilter.calculate(
                                (currentSim.getRightVelocityMetersPerSecond()
                                                + currentSim.getLeftVelocityMetersPerSecond())
                                        / 2,
                                TimedRobot.kDefaultPeriod),
                        0,
                        0);

        Arm.getInstance()
                .setSimState(
                        UtilityMath.normalizeAngleRadians(armState.get(0, 0)),
                        UtilityMath.normalizeAngleRadians(armState.get(0, 1)));

        PowerDistributionHub.getInstance().setSimState(totalCurrentDraw);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDraw));

        DifferentialDrivetrainSim notCurrentSim =
                Drivetrain.getInstance().getShifterSlow() ? fastDrivetrainSim : slowDrivetrainSim;

        notCurrentSim.setState(
                VecBuilder.fill(
                        currentSim.getPose().getX(),
                        currentSim.getPose().getY(),
                        currentSim.getHeading().getRadians(),
                        currentSim.getLeftVelocityMetersPerSecond(),
                        currentSim.getRightVelocityMetersPerSecond(),
                        currentSim.getLeftPositionMeters(),
                        currentSim.getRightPositionMeters()));
    }
}
