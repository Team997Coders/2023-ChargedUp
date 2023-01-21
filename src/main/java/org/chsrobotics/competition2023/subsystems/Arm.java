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
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.telemetry.Logger;

public class Arm implements Subsystem {
    private static final Arm instance = new Arm();

    private final CANSparkMax distalNEO =
            new CANSparkMax(Constants.SUBSYSTEM.ARM.DISTAL_NEO_CAN_ID, MotorType.kBrushless);

    private final CANSparkMax localNEOa =
            new CANSparkMax(Constants.SUBSYSTEM.ARM.LOCAL_NEO_A_CAN_ID, MotorType.kBrushless);

    private final CANSparkMax localNEOb =
            new CANSparkMax(Constants.SUBSYSTEM.ARM.LOCAL_NEO_B_CAN_ID, MotorType.kBrushless);

    private final AnalogEncoder distalPotentiometer =
            new AnalogEncoder(Constants.SUBSYSTEM.ARM.DISTAL_POTENTIOMETER_ANALOG_CHANNEL);

    private final AnalogEncoder localPotentiometer =
            new AnalogEncoder(Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_ANALOG_CHANNEL);

    private final DifferentiatingFilter localAccelFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter distalAccelFilter = new DifferentiatingFilter();

    private final String subdirString = "arm";

    private final Logger<Double> distalPositionLogger =
            new Logger<>("distalPosition_radians", subdirString);
    private final Logger<Double> localPositionLogger =
            new Logger<>("localPosition_radians", subdirString);
    private final Logger<Double> distalVelocityLogger =
            new Logger<>("distalVelocity_radiansPerSecond", subdirString);
    private final Logger<Double> localVelocityLogger =
            new Logger<>("localVelocity_radiansPerSecond", subdirString);
    private final Logger<Double> distalAccelLogger =
            new Logger<>("distalAccel_radiansPerSecondSquared", subdirString);
    private final Logger<Double> localAccelLogger =
            new Logger<>("localAccel_radiansPerSecondSqaured", subdirString);
    private final Logger<Double> distalNeoCurrentLogger =
            new Logger<>("distalNeoCurrent_amps", subdirString);
    private final Logger<Double> localNeoACuCrrentLogger =
            new Logger<>("localNeoACurrent_amps", subdirString);
    private final Logger<Double> localNeoBCurrentLogger =
            new Logger<>("localNeoBCurrent_amps", subdirString);
    private final Logger<Double> distalNeoTempLogger =
            new Logger<>("distalNeoTemp_C", subdirString);
    private final Logger<Double> localNeoATempLogger =
            new Logger<>("localNeoATemp_C", subdirString);
    private final Logger<Double> localNeoBTempLogger =
            new Logger<>("localNeoBTemp_c", subdirString);

    private double localVelocity = 0;
    private double distalVelocity = 0;

    private double previousDistalPosition = 0;
    private double previousLocalPosition = 0;

    private Arm() {
        register();

        distalNEO.setInverted(Constants.SUBSYSTEM.ARM.DISTAL_NEO_INVERTED);
        localNEOa.setInverted(Constants.SUBSYSTEM.ARM.LOCAL_NEO_A_INVERTED);
        localNEOb.setInverted(Constants.SUBSYSTEM.ARM.LOCAL_NEO_B_INVERTED);
    }

    public double getDistalPotentiometerAngleRadians() {
        ;
        return Constants.SUBSYSTEM.ARM.DISTAL_POTENTIOMTER_CONVERSION_HELPER.outputFromInput(
                        distalPotentiometer.getAbsolutePosition())
                - Constants.SUBSYSTEM.ARM.DISTAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO;
    }

    public double getLocalPotentiometerAngleRadians() {
        return Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_CONVERSION_HELPER.outputFromInput(
                        localPotentiometer.getAbsolutePosition())
                - Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO;
    }

    public void setDistalVoltage(double voltage) {
        distalNEO.setVoltage(voltage);
    }

    public void setLocalVoltage(double voltage) {
        localNEOa.setVoltage(voltage);
        localNEOb.setVoltage(voltage);
    }

    public void setDistalIdleMode(boolean trueIfCoast) {
        distalNEO.setIdleMode(trueIfCoast ? IdleMode.kCoast : IdleMode.kBrake);
    }

    public void setLocalIdleMode(boolean trueIfCoast) {
        localNEOa.setIdleMode(trueIfCoast ? IdleMode.kCoast : IdleMode.kBrake);
        localNEOb.setIdleMode(trueIfCoast ? IdleMode.kCoast : IdleMode.kBrake);
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
        double distalDeltaPosition = getDistalPotentiometerAngleRadians() - previousDistalPosition;
        double localDeltaPostion = getLocalPotentiometerAngleRadians() - previousLocalPosition;

        distalVelocity = distalDeltaPosition / 0.02;
        localVelocity = localDeltaPostion / 0.02;

        localAccelFilter.calculate(localVelocity);
        distalAccelFilter.calculate(distalVelocity);

        previousDistalPosition = getDistalPotentiometerAngleRadians();
        previousLocalPosition = getLocalPotentiometerAngleRadians();

        distalPositionLogger.update(getDistalPotentiometerAngleRadians());
        localPositionLogger.update(getLocalPotentiometerAngleRadians());
        distalVelocityLogger.update(getDistalVelocityRadiansPerSecond());
        localVelocityLogger.update(getLocalVelociyRadiansPerSecond());
        distalAccelLogger.update(getDistalAccelRadiansPerSecondSquared());
        localAccelLogger.update(getLocalAccelRadiansPerSecondSquared());
        distalNeoCurrentLogger.update(distalNEO.getOutputCurrent());
        localNeoACuCrrentLogger.update(localNEOa.getOutputCurrent());
        localNeoBCurrentLogger.update(localNEOb.getOutputCurrent());
        distalNeoTempLogger.update(distalNEO.getMotorTemperature());
        localNeoATempLogger.update(localNEOa.getMotorTemperature());
        localNeoBTempLogger.update(localNEOb.getMotorTemperature());
    }

    public static Arm getInstance() {
        return instance;
    }
}
