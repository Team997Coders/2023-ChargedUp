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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Robot;
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

    private final Solenoid leftShifter =
            Pneumatics.getInstance()
                    .getSolenoid(Constants.SUBSYSTEM.DRIVETRAIN.LEFT_SHIFTER_SOLENOID_CHANNEL);
    private final Solenoid rightShifter =
            Pneumatics.getInstance()
                    .getSolenoid(Constants.SUBSYSTEM.DRIVETRAIN.RIGHT_SHIFTER_SOLENOID_CHANNEL);

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
    private final Logger<Boolean> isLeftShifterOnLogger =
            new Logger<>("isLeftShifterOn", subdirString);
    private final Logger<Boolean> isRightShifterOnLogger =
            new Logger<>("isRightShifterOn", subdirString);

    private Drivetrain() {
        register();
        frontLeftSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.FRONT_LEFT_IS_INVERTED);
        frontRightSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.FRONT_RIGHT_IS_INVERTED);
        backLeftSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.BACK_LEFT_IS_INVERTED);
        backRightSparkMax.setInverted(Constants.SUBSYSTEM.DRIVETRAIN.BACK_RIGHT_IS_INVERTED);
    }

    public void setRightVoltages(double voltage) {
        frontRightSparkMax.setVoltage(voltage);
        backRightSparkMax.setVoltage(voltage);
        rightSetVoltageLogger.update(voltage);
    }

    public void setLeftVoltages(double voltage) {
        frontLeftSparkMax.setVoltage(voltage);
        backLeftSparkMax.setVoltage(voltage);
        leftSetVoltageLogger.update(voltage);
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

    public void setLeftShifterOn(boolean isLeftShifterOn) {
        if (Constants.SUBSYSTEM.DRIVETRAIN.LEFT_SHIFTER_SOLENOID_IS_INVERTED) {
            leftShifter.set(!isLeftShifterOn);
            isLeftShifterOnLogger.update(!isLeftShifterOn);
        } else {
            leftShifter.set(isLeftShifterOn);
            isLeftShifterOnLogger.update(isLeftShifterOn);
        }
    }

    public void setRightShifterOn(boolean isRightShifterOn) {
        if (Constants.SUBSYSTEM.DRIVETRAIN.RIGHT_SHIFTER_SOLENOID_IS_INVERTED) {
            rightShifter.set(!isRightShifterOn);
            isRightShifterOnLogger.update(!isRightShifterOn);
        } else {
            rightShifter.set(isRightShifterOn);
            isRightShifterOnLogger.update(isRightShifterOn);
        }
    }

    public boolean getLeftShifterSlow() {

        if (Constants.SUBSYSTEM.DRIVETRAIN.LEFT_SHIFTER_SOLENOID_IS_INVERTED) {
            return !leftShifter.get();
        } else {
            return leftShifter.get();
        }
    }

    public boolean getRightShifterSlow() {

        if (Constants.SUBSYSTEM.DRIVETRAIN.RIGHT_SHIFTER_SOLENOID_IS_INVERTED) {
            return !rightShifter.get();
        } else {
            return rightShifter.get();
        }
    }

    public double getFrontRightSensorPosition() {
        return metersFromNEORotations(frontRightSparkMax.getEncoder().getPosition());
    }

    public double getBackRightSensorPosition() {
        return metersFromNEORotations(backRightSparkMax.getEncoder().getPosition());
    }

    public double getFrontLeftSensorPosition() {
        return metersFromNEORotations(frontLeftSparkMax.getEncoder().getPosition());
    }

    public double getbackLeftSensorPosition() {
        return metersFromNEORotations(backLeftSparkMax.getEncoder().getPosition());
    }

    private double metersFromNEORotations(double rotations) {
        if (getRightShifterSlow()) {
            return Constants.SUBSYSTEM.DRIVETRAIN.SLOW_GEAR_RATIO.outputFromInput(rotations)
                    * 2
                    * Math.PI
                    * Constants.SUBSYSTEM.DRIVETRAIN.WHEEL_RADIUS_METERS;
        } else {
            return Constants.SUBSYSTEM.DRIVETRAIN.FAST_GEAR_RATIO.outputFromInput(rotations)
                    * 2
                    * Math.PI
                    * Constants.SUBSYSTEM.DRIVETRAIN.WHEEL_RADIUS_METERS;
        }
    }

    public static Drivetrain getInstance() {
        return instance;
    }

    public void setSimState(double leftMeters, double rightMeters) {
        if (!Robot.isReal()) {
        } else {
            HighLevelLogger.getInstance()
                    .logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.getInstance()
                    .logWarning("There might be sim code still running somewhere!");
        }
    }

    @Override
    public void periodic() {
        frontLeftTemperatureLogger.update(frontLeftSparkMax.getMotorTemperature());
        frontRightTemperatureLogger.update(frontRightSparkMax.getMotorTemperature());
        backLeftTemperatureLogger.update(backLeftSparkMax.getMotorTemperature());
        backRightTemperatureLogger.update(backRightSparkMax.getMotorTemperature());
    }
}
