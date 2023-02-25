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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Robot;
import org.chsrobotics.competition2023.Simulation;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.telemetry.Logger;

public class Drivetrain implements Subsystem {
    private static Drivetrain instance = new Drivetrain();

    private final CANSparkMax frontRightSparkMax =
            new CANSparkMax(
                    Constants.SUBSYSTEM.DRIVETRAIN.FRONT_RIGHT_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax backRightSparkMax =
            new CANSparkMax(Constants.SUBSYSTEM.DRIVETRAIN.BACK_RIGHT_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax frontLeftSparkMax =
            new CANSparkMax(Constants.SUBSYSTEM.DRIVETRAIN.FRONT_LEFT_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax backLeftSparkMax =
            new CANSparkMax(Constants.SUBSYSTEM.DRIVETRAIN.BACK_LEFT_CAN_ID, MotorType.kBrushless);

    private final Encoder leftEncoder =
            new Encoder(
                    Constants.SUBSYSTEM.DRIVETRAIN.LEFT_ENCODER_CHANNEL_A,
                    Constants.SUBSYSTEM.DRIVETRAIN.LEFT_ENCODER_CHANNEL_B);

    private final Encoder rightEncoder =
            new Encoder(
                    Constants.SUBSYSTEM.DRIVETRAIN.RIGHT_ENCODER_CHANNEL_A,
                    Constants.SUBSYSTEM.DRIVETRAIN.RIGHT_ENCODER_CHANNEL_B);

    private final Solenoid shifter =
            Pneumatics.getInstance()
                    .getSolenoid(Constants.SUBSYSTEM.DRIVETRAIN.SHIFTER_SOLENOID_CHANNEL);

    private final DifferentiatingFilter rightVelocityFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter leftVelocityFilter = new DifferentiatingFilter();

    private final DifferentiatingFilter rightAccelerationFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter leftAccelerationFilter = new DifferentiatingFilter();

    private final String subdirString = "drivetrain";

    private final Logger<Double> leftSetVoltageLogger =
            new Logger<>("leftSetVoltage_volts", subdirString);
    private final Logger<Double> rightSetVoltageLogger =
            new Logger<>("rightSetVoltage_volts", subdirString);

    private final Logger<Boolean> isCoastModeLogger = new Logger<>("isCoastMode", subdirString);

    private final Logger<Double> frontRightTemperatureLogger =
            new Logger<>("frontRightTemperature_C", subdirString);
    private final Logger<Double> frontLeftTemperatureLogger =
            new Logger<>("frontLeftTemperature_C", subdirString);
    private final Logger<Double> backRightTemperatureLogger =
            new Logger<>("backRightTemperature_C", subdirString);
    private final Logger<Double> backLeftTemperatureLogger =
            new Logger<>("backLeftTemperature_C", subdirString);

    private final Logger<Boolean> shifterOnLogger = new Logger<>("shifterSlow", subdirString);

    private final Logger<Double> rightPositionLogger =
            new Logger<>("rightSensorPosition_meters", subdirString);
    private final Logger<Double> leftPositionLogger =
            new Logger<>("leftSensorPosition_meters", subdirString);

    private final Logger<Double> rightVelocityLogger =
            new Logger<>("rightVelocity_mps", subdirString);
    private final Logger<Double> leftVelocityLogger =
            new Logger<>("leftVeloctiy_mps", subdirString);

    private final Logger<Double> rightAccelerationLogger =
            new Logger<>("rightAcceleration_mpsSquared", subdirString);
    private final Logger<Double> leftAccelerationLogger =
            new Logger<>("leftAcceleration_mpsSquared", subdirString);

    private final Logger<Double> frontRightCurrentLogger =
            new Logger<>("frontRightCurrent_amps", subdirString);
    private final Logger<Double> backRightCurrentLogger =
            new Logger<>("backRightCurrent_amps", subdirString);
    private final Logger<Double> frontLeftCurrentLogger =
            new Logger<>("frontLeftCurrent_amps", subdirString);
    private final Logger<Double> backLeftCurrentLogger =
            new Logger<>("backLeftCurrent_amps", subdirString);

    private boolean shiftersInSlow = true;

    private double setLeftVoltage = 0;
    private double setRightVoltage = 0;

    private double simLeftPositionMeters = 0;
    private double simRightPositionMeters = 0;

    private Drivetrain() {
        register();
        frontLeftSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.FRONT_LEFT_IS_INVERTED);
        frontRightSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.FRONT_RIGHT_IS_INVERTED);
        backLeftSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.BACK_LEFT_IS_INVERTED);
        backRightSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.BACK_RIGHT_IS_INVERTED);

        setShifters(shiftersInSlow);
    }

    public static Drivetrain getInstance() {
        return instance;
    }

    public void setRightVoltages(double voltage) {
        frontRightSparkMax.setVoltage(voltage);
        backRightSparkMax.setVoltage(voltage);
        rightSetVoltageLogger.update(voltage);

        setRightVoltage = voltage;
    }

    public void setLeftVoltages(double voltage) {
        frontLeftSparkMax.setVoltage(voltage);
        backLeftSparkMax.setVoltage(voltage);
        leftSetVoltageLogger.update(voltage);

        setLeftVoltage = voltage;
    }

    public void setBrakeMode(boolean isCoastMode) {
        frontRightSparkMax.setIdleMode(isCoastMode ? IdleMode.kCoast : IdleMode.kBrake);
        backRightSparkMax.setIdleMode(isCoastMode ? IdleMode.kCoast : IdleMode.kBrake);
        frontLeftSparkMax.setIdleMode(isCoastMode ? IdleMode.kCoast : IdleMode.kBrake);
        backLeftSparkMax.setIdleMode(isCoastMode ? IdleMode.kCoast : IdleMode.kBrake);
        isCoastModeLogger.update(isCoastMode);
    }

    public boolean getIsCoastMode() {
        return (frontLeftSparkMax.getIdleMode() == IdleMode.kCoast);
    }

    public void setShifters(boolean slow) {
        if (Constants.SUBSYSTEM.DRIVETRAIN.SHIFTER_SOLENOID_INVERTED) {
            shifter.set(!slow);
            shifterOnLogger.update(!slow);
            shiftersInSlow = false;
        } else {
            shifter.set(slow);
            shifterOnLogger.update(slow);
            shiftersInSlow = true;
        }
    }

    public boolean getShifterSlow() {
        return shiftersInSlow;
    }

    public double getRightSensorPosition() {
        if (Robot.isReal()) {
            return Constants.SUBSYSTEM.DRIVETRAIN.ENCODER_TO_OUTPUT.outputFromInput(
                            rightEncoder.get())
                    * 2
                    * Math.PI
                    * Constants.SUBSYSTEM.DRIVETRAIN.WHEEL_RADIUS_METERS;
        } else {
            return simRightPositionMeters;
        }
    }

    public double getLeftSensorPosition() {
        if (Robot.isReal()) {
            return Constants.SUBSYSTEM.DRIVETRAIN.ENCODER_TO_OUTPUT.outputFromInput(
                            leftEncoder.get())
                    * 2
                    * Math.PI
                    * Constants.SUBSYSTEM.DRIVETRAIN.WHEEL_RADIUS_METERS;
        } else {
            return simLeftPositionMeters;
        }
    }

    public double getLeftSideVelocity() {
        return leftVelocityFilter.getCurrentOutput();
    }

    public double getRightSideVelocity() {
        return rightVelocityFilter.getCurrentOutput();
    }

    public double getLeftSideAcceleration() {
        return leftAccelerationFilter.getCurrentOutput();
    }

    public double getRightSideAcceleration() {
        return rightAccelerationFilter.getCurrentOutput();
    }

    public void setSimState(double leftMeters, double rightMeters) {
        if (!Robot.isReal()) {
            simLeftPositionMeters = leftMeters;
            simRightPositionMeters = rightMeters;
        } else {
            HighLevelLogger.getInstance()
                    .logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.getInstance()
                    .logWarning("There might be sim code still running somewhere!");
        }
    }

    @Override
    public void periodic() {
        Simulation.getInstance().setDrivetrainInputs(setLeftVoltage, setRightVoltage);

        frontLeftTemperatureLogger.update(frontLeftSparkMax.getMotorTemperature());
        frontRightTemperatureLogger.update(frontRightSparkMax.getMotorTemperature());
        backLeftTemperatureLogger.update(backLeftSparkMax.getMotorTemperature());
        backRightTemperatureLogger.update(backRightSparkMax.getMotorTemperature());

        frontRightCurrentLogger.update(frontRightSparkMax.getOutputCurrent());
        backRightCurrentLogger.update(backRightSparkMax.getOutputCurrent());
        frontLeftCurrentLogger.update(frontLeftSparkMax.getOutputCurrent());
        backLeftCurrentLogger.update(backLeftSparkMax.getOutputCurrent());

        rightPositionLogger.update(getRightSensorPosition());
        leftPositionLogger.update(getLeftSensorPosition());

        rightVelocityLogger.update(rightVelocityFilter.calculate(getRightSensorPosition()));
        leftVelocityLogger.update(leftVelocityFilter.calculate(getLeftSensorPosition()));

        rightAccelerationLogger.update(
                rightAccelerationFilter.calculate(rightVelocityFilter.getCurrentOutput()));
        leftAccelerationLogger.update(
                leftAccelerationFilter.calculate(leftVelocityFilter.getCurrentOutput()));
    }
}
