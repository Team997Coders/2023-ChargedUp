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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.lib.telemetry.Logger;

public class Intake implements Subsystem {
    private static final Intake instance = new Intake();

    private final CANSparkMax leftMotor =
            new CANSparkMax(Constants.SUBSYSTEM.INTAKE.LEFT_MOTOR_CANID, MotorType.kBrushless);
    private final CANSparkMax rightMotor =
            new CANSparkMax(Constants.SUBSYSTEM.INTAKE.RIGHT_MOTOR_CANID, MotorType.kBrushless);

    private final Solenoid leftDeploySolenoid =
            Pneumatics.getInstance()
                    .getSolenoid(Constants.SUBSYSTEM.INTAKE.LEFT_DEPLOY_SOLENOID_CHANNEL);
    private final Solenoid rightDeploySolenoid =
            Pneumatics.getInstance()
                    .getSolenoid(Constants.SUBSYSTEM.INTAKE.RIGHT_DEPLOY_SOLENOID_CHANNEL);

    private final Timer timeBuffer = new Timer();

    private IntakeState currentState = IntakeState.RETRACTED;
    private IntakeState desiredState = IntakeState.RETRACTED;

    private double desiredVoltage = 0;

    private final String subdirString = "intake";

    private final Logger<String> desiredStateLogger = new Logger<>("desiredState", subdirString);
    private final Logger<String> actualStateLogger = new Logger<>("actualState", subdirString);

    private final Logger<Boolean> deployedLogger = new Logger<>("intakeDeployed", subdirString);

    private final Logger<Double> appliedVoltageLogger =
            new Logger<>("appliedVoltage_v", subdirString);

    private final Logger<Double> rightTempLogger = new Logger<>("rightMotorTemp_C", subdirString);
    private final Logger<Double> leftTempLogger = new Logger<>("leftMotorTemp_C", subdirString);

    private final Logger<Double> rightCurrentLogger =
            new Logger<>("rightMotorCurrent_a", subdirString);
    private final Logger<Double> leftCurrentLogger =
            new Logger<>("leftMotorCurrent_a", subdirString);

    private enum IntakeState {
        RETRACTED,
        EXTENDED_BUFFER_WAIT,
        EXTENDED_MOTOR_RUN
    }

    private Intake() {
        register();

        leftMotor.setInverted(Constants.SUBSYSTEM.INTAKE.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(Constants.SUBSYSTEM.INTAKE.RIGHT_MOTOR_INVERTED);

        timeBuffer.reset();
        timeBuffer.stop();
    }

    public static Intake getInstance() {
        return instance;
    }

    public void setVoltage(double voltage) {
        desiredVoltage = voltage;
    }

    public void setDeployed(boolean deploy) {
        if (deploy) desiredState = IntakeState.EXTENDED_MOTOR_RUN;
        else desiredState = IntakeState.RETRACTED;
    }

    public boolean isDeployed() {
        return (currentState != IntakeState.RETRACTED);
    }

    public boolean motorsRunning() {
        return (currentState == IntakeState.EXTENDED_MOTOR_RUN);
    }

    private void setSolenoids(boolean deployed) {
        if (deployed) {
            rightDeploySolenoid.set(!Constants.SUBSYSTEM.INTAKE.RIGHT_DEPLOY_SOLENOID_INVERTED);
            leftDeploySolenoid.set(!Constants.SUBSYSTEM.INTAKE.LEFT_DEPLOY_SOLENOID_INVERTED);

            deployedLogger.update(true);
        } else {
            rightDeploySolenoid.set(Constants.SUBSYSTEM.INTAKE.RIGHT_DEPLOY_SOLENOID_INVERTED);
            leftDeploySolenoid.set(Constants.SUBSYSTEM.INTAKE.LEFT_DEPLOY_SOLENOID_INVERTED);

            deployedLogger.update(false);
        }
    }

    private void setMotorVoltages(double voltage) {
        rightMotor.setVoltage(voltage);
        leftMotor.setVoltage(voltage);
        appliedVoltageLogger.update(voltage);
    }

    @Override
    public void periodic() {
        if (currentState == IntakeState.RETRACTED
                && desiredState == IntakeState.EXTENDED_MOTOR_RUN) {
            currentState = IntakeState.EXTENDED_BUFFER_WAIT;
            timeBuffer.reset();
            timeBuffer.start();
        } else if (currentState == IntakeState.EXTENDED_BUFFER_WAIT
                && desiredState == IntakeState.EXTENDED_MOTOR_RUN) {
            if (timeBuffer.hasElapsed(
                    Constants.SUBSYSTEM.INTAKE.DEPLOY_RUN_MOTORS_TIME_BUFFER_SECONDS)) {
                currentState = IntakeState.EXTENDED_MOTOR_RUN;
            }
        } else if ((currentState == IntakeState.EXTENDED_MOTOR_RUN
                        || currentState == IntakeState.EXTENDED_BUFFER_WAIT)
                && desiredState == IntakeState.RETRACTED) {
            currentState = IntakeState.RETRACTED;
        }

        if (currentState == IntakeState.RETRACTED) {
            setSolenoids(false);
            setMotorVoltages(0);
        } else if (currentState == IntakeState.EXTENDED_BUFFER_WAIT) {
            setSolenoids(true);
            setMotorVoltages(0);
        } else {
            setSolenoids(true);
            setMotorVoltages(desiredVoltage);
        }

        desiredStateLogger.update(desiredState.toString());
        actualStateLogger.update(currentState.toString());

        rightTempLogger.update(rightMotor.getMotorTemperature());
        leftTempLogger.update(leftMotor.getMotorTemperature());

        rightCurrentLogger.update(rightMotor.getOutputCurrent());
        leftCurrentLogger.update(leftMotor.getOutputCurrent());
    }
}
