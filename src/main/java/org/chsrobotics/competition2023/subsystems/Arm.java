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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Robot;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.math.filters.ExponentialMovingAverage;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;

public class Arm implements Subsystem {
    private static final Arm instance = new Arm();

    private final CANSparkMax distalNEO =
            new CANSparkMax(Constants.SUBSYSTEM.ARM.DISTAL_NEO_CAN_ID, MotorType.kBrushless);

    private final CANSparkMax leftLocalNEO =
            new CANSparkMax(Constants.SUBSYSTEM.ARM.LEFT_LOCAL_NEO_CAN_ID, MotorType.kBrushless);

    private final CANSparkMax rightLocalNEO =
            new CANSparkMax(Constants.SUBSYSTEM.ARM.RIGHT_LOCAL_NEO_CAN_ID, MotorType.kBrushless);

    private final AnalogPotentiometer distalPotentiometer =
            new AnalogPotentiometer(Constants.SUBSYSTEM.ARM.DISTAL_POTENTIOMETER_ANALOG_CHANNEL);

    private final AnalogPotentiometer localPotentiometer =
            new AnalogPotentiometer(Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_ANALOG_CHANNEL);

    private final ExponentialMovingAverage localAngleSmoother =
            new ExponentialMovingAverage(
                    Constants.SUBSYSTEM.ARM.LOCAL_ANGLE_SMOOTHING_RESPONSE_CONSTANT);

    private final ExponentialMovingAverage distalAngleSmoother =
            new ExponentialMovingAverage(
                    Constants.SUBSYSTEM.ARM.DISTAL_ANGLE_SMOOTHING_RESPONSE_CONSTANT);

    private final DifferentiatingFilter localAccelFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter distalAccelFilter = new DifferentiatingFilter();

    private final String subdirString = "arm";

    private final LoggerFactory<Double> factory = new LoggerFactory<>(subdirString);

    private final Logger<Double> distalEncoderPositionLogger =
            factory.getLogger("distalEncoderPosition_radians");
    private final Logger<Double> localEncoderPositionLogger =
            factory.getLogger("localEncoderPosition_radians");

    private final Logger<Double> distalPotPositionLogger =
            factory.getLogger("distalPotPosition_radians");
    private final Logger<Double> localPotPositionLogger =
            factory.getLogger("localPotPosition_radians");

    private final Logger<Double> distalFusedPositionLogger =
            factory.getLogger("distalFusedPosition_radians");
    private final Logger<Double> localFusedPositionLogger =
            factory.getLogger("localFusedPosition_radians");

    private final Logger<Double> distalVelocityLogger =
            factory.getLogger("distalVelocity_radiansPerSecond");
    private final Logger<Double> localVelocityLogger =
            factory.getLogger("localVelocity_radiansPerSecond");

    private final Logger<Double> distalAccelLogger =
            factory.getLogger("distalAccel_radiansPerSecondSquared");
    private final Logger<Double> localAccelLogger =
            factory.getLogger("localAccel_radiansPerSecondSqaured");

    private final Logger<Double> distalNeoCurrentLogger =
            factory.getLogger("distalNeoCurrent_amps");
    private final Logger<Double> leftLocalNEOCurrentLogger =
            factory.getLogger("localNeoACurrent_amps");
    private final Logger<Double> rightLocalNEOCurrentLogger =
            factory.getLogger("localNeoBCurrent_amps");

    private final Logger<Double> distalNeoTempLogger = factory.getLogger("distalNeoTemp_C");
    private final Logger<Double> leftLocalNEOTempLogger = factory.getLogger("localNeoATemp_C");
    private final Logger<Double> rightLocalNEOTempLogger = factory.getLogger("localNeoBTemp_c");

    private double localVelocity = 0;
    private double distalVelocity = 0;

    private double previousDistalPosition = 0;
    private double previousLocalPosition = 0;

    private final double initLocalPosition;
    private final double initDistalPosition;

    private double simLocalAngle = 0;
    private double simDistalAngle = 0;

    private Arm() {
        register();

        distalNEO.setInverted(Constants.SUBSYSTEM.ARM.DISTAL_NEO_INVERTED);
        leftLocalNEO.setInverted(Constants.SUBSYSTEM.ARM.LEFT_LOCAL_NEO_INVERTED);
        rightLocalNEO.setInverted(Constants.SUBSYSTEM.ARM.RIGHT_LOCAL_NEO_INVERTED);

        distalNEO.setIdleMode(Constants.SUBSYSTEM.ARM.IDLE_MODE);
        leftLocalNEO.setIdleMode(Constants.SUBSYSTEM.ARM.IDLE_MODE);
        rightLocalNEO.setIdleMode(Constants.SUBSYSTEM.ARM.IDLE_MODE);

        initLocalPosition = getLocalAngleRadians();
        initDistalPosition = getDistalAngleRadians();

        factory.getLogger("initLocalPosition_radians").update(initLocalPosition);
        factory.getLogger("initDistalPosition_radians").update(initDistalPosition);
    }

    public double getDistalAngleRadians() {
        if (Robot.isReal()) {
            return distalAngleSmoother.getCurrentOutput();
        } else return simDistalAngle;
    }

    public double getLocalAngleRadians() {
        if (Robot.isReal()) {
            return localAngleSmoother.getCurrentOutput();
        } else return simLocalAngle;
    }

    private double getPotDistalAngle() {
        return Constants.SUBSYSTEM.ARM.DISTAL_POTENTIOMETER_CONVERSION_HELPER.outputFromInput(
                        distalPotentiometer.get()
                                * Constants.SUBSYSTEM.ARM.POTENTIOMETER_RANGE_RADIANS)
                - Constants.SUBSYSTEM.ARM.DISTAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO;
    }

    private double getPotLocalAngle() {
        return Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_CONVERSION_HELPER.outputFromInput(
                        localPotentiometer.get()
                                * Constants.SUBSYSTEM.ARM.POTENTIOMETER_RANGE_RADIANS)
                - Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO;
    }

    private double getEncoderDistalAngle() {
        return Constants.SUBSYSTEM.ARM.DISTAL_MOTOR_CONVERSION_HELPER.outputFromInput(
                        distalNEO.getEncoder().getPosition())
                + initDistalPosition;
    }

    private double getEncoderLocalAngle() {
        double avg =
                UtilityMath.arithmeticMean(
                        new double[] {
                            leftLocalNEO.getEncoder().getPosition(),
                            rightLocalNEO.getEncoder().getPosition()
                        });

        return Constants.SUBSYSTEM.ARM.LOCAL_MOTORS_CONVERSION_HELPER.outputFromInput(avg)
                + initLocalPosition;
    }

    public void setVoltages(double localVoltage, double distalVoltage) {
        distalNEO.setVoltage(distalVoltage);

        leftLocalNEO.setVoltage(localVoltage);
        rightLocalNEO.setVoltage(localVoltage);
    }

    public double getDistalVelocityRadiansPerSecond() {
        return distalVelocity;
    }

    public double getLocalVelociyRadiansPerSecond() {
        return localVelocity;
    }

    public double getLocalAccelRadiansPerSecondSquared() {
        return localAccelFilter.getCurrentOutput();
    }

    public double getDistalAccelRadiansPerSecondSquared() {
        return distalAccelFilter.getCurrentOutput();
    }

    @Override
    public void periodic() {
        localAngleSmoother.calculate(
                UtilityMath.arithmeticMean(
                        new double[] {getPotLocalAngle(), getEncoderLocalAngle()}));

        distalAngleSmoother.calculate(
                UtilityMath.arithmeticMean(
                        new double[] {getPotDistalAngle(), getEncoderDistalAngle()}));

        double distalDeltaPosition;

        if (previousDistalPosition < getDistalAngleRadians()) {
            distalDeltaPosition =
                    UtilityMath.smallestAngleRadiansBetween(
                            previousDistalPosition, getDistalAngleRadians());
        } else {
            distalDeltaPosition =
                    UtilityMath.smallestAngleRadiansBetween(
                            getDistalAngleRadians(), previousDistalPosition);
        }

        double localDeltaPostion;

        if (previousLocalPosition < getLocalAngleRadians()) {
            localDeltaPostion =
                    UtilityMath.smallestAngleRadiansBetween(
                            previousLocalPosition, getLocalAngleRadians());
        } else {
            localDeltaPostion =
                    UtilityMath.smallestAngleRadiansBetween(
                            getLocalAngleRadians(), previousLocalPosition);
        }

        distalVelocity = distalDeltaPosition / 0.02;
        localVelocity = localDeltaPostion / 0.02;

        localAccelFilter.calculate(localVelocity);
        distalAccelFilter.calculate(distalVelocity);

        previousDistalPosition = getDistalAngleRadians();
        previousLocalPosition = getLocalAngleRadians();

        distalFusedPositionLogger.update(getDistalAngleRadians());
        localFusedPositionLogger.update(getLocalAngleRadians());

        distalEncoderPositionLogger.update(getEncoderDistalAngle());
        localEncoderPositionLogger.update(getEncoderLocalAngle());

        distalPotPositionLogger.update(getPotDistalAngle());
        localPotPositionLogger.update(getPotLocalAngle());

        distalVelocityLogger.update(getDistalVelocityRadiansPerSecond());
        localVelocityLogger.update(getLocalVelociyRadiansPerSecond());

        distalAccelLogger.update(getDistalAccelRadiansPerSecondSquared());
        localAccelLogger.update(getLocalAccelRadiansPerSecondSquared());

        distalNeoCurrentLogger.update(distalNEO.getOutputCurrent());
        leftLocalNEOCurrentLogger.update(leftLocalNEO.getOutputCurrent());
        rightLocalNEOCurrentLogger.update(rightLocalNEO.getOutputCurrent());

        distalNeoTempLogger.update(distalNEO.getMotorTemperature());
        leftLocalNEOTempLogger.update(leftLocalNEO.getMotorTemperature());
        rightLocalNEOTempLogger.update(rightLocalNEO.getMotorTemperature());
    }

    public static Arm getInstance() {
        return instance;
    }

    public void setSimState(double localAngleRadians, double distalAngleRadians) {
        if (!Robot.isReal()) {
            simLocalAngle = localAngleRadians;
            simDistalAngle = distalAngleRadians;
        } else {
            HighLevelLogger.getInstance()
                    .logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.getInstance()
                    .logWarning("There might be sim code still running somewhere!");
        }
    }
}
