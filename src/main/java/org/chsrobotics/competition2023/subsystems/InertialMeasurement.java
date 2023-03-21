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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Robot;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.telemetry.Logger;

public class InertialMeasurement implements Subsystem {
    private static final InertialMeasurement instance = new InertialMeasurement();

    private AHRS navX;

    private boolean hasStartedCalibration = false;

    private double previousPitchRadians = 0;
    private double previousYawRadians = 0;
    private double previousRollRadians = 0;

    private double pitchVelocity = 0;
    private double yawVelocity = 0;
    private double rollVelocity = 0;

    private final DifferentiatingFilter pitchAFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter yawAFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter rollAFilter = new DifferentiatingFilter();

    private final String subdirString = "inertialMeasurement";

    private final Logger<Boolean> isCalibratingLogger = new Logger<>("isCalibrating", subdirString);
    private final Logger<Boolean> hasCalibratedLogger = new Logger<>("hasCalibrated", subdirString);

    private final Logger<Double> pitchLogger = new Logger<>("pitch_rad", subdirString);
    private final Logger<Double> yawLogger = new Logger<>("yaw_rad", subdirString);
    private final Logger<Double> rollLogger = new Logger<>("roll_rad", subdirString);

    private final Logger<Double> pitchVLogger = new Logger<>("pitchVelocity_rad_p_s", subdirString);
    private final Logger<Double> yawVLogger =
            new Logger<>("yawVelocityLogger_rad_p_s", subdirString);
    private final Logger<Double> rollVLogger = new Logger<>("rollVelocity_rad_p_s", subdirString);

    private final Logger<Double> pitchALogger =
            new Logger<>("pitchAcceleration_rad_p_s^2", subdirString);
    private final Logger<Double> yawALogger =
            new Logger<>("yawAcceleratoin_rad_p_s^2", subdirString);
    private final Logger<Double> rollALogger =
            new Logger<>("rollAcceleration_rad_p_s^2", subdirString);

    private final Logger<Double> xALogger = new Logger<>("xAcceleration_m_p_s^2", subdirString);
    private final Logger<Double> yALogger = new Logger<>("yAcceleration_m_p_s^2", subdirString);
    private final Logger<Double> zALogger = new Logger<>("zAcceleration_m_p_s^2", subdirString);

    private final Logger<Double> temperatureLogger = new Logger<>("temperature_C", subdirString);

    private double simPitchRadians = 0;
    private double simYawRadians = 0;
    private double simRollRadians = 0;

    private double simXAccel = 0;
    private double simYAccel = 0;
    private double simZAccel = 0;

    private InertialMeasurement() {
        register();

        if (Robot.isReal()) {
            navX = new AHRS();
        }
    }

    public static InertialMeasurement getInstance() {
        return instance;
    }

    public void startCalibration() {
        hasStartedCalibration = true;
        if (Robot.isReal()) {
            navX.calibrate();
        }
    }

    public boolean isCalibrating() {
        if (Robot.isReal()) {
            return navX.isCalibrating();
        } else {
            return false;
        }
    }

    public boolean hasCalibrated() {
        return (hasStartedCalibration && !isCalibrating());
    }

    public void reset() {
        previousPitchRadians = 0;
        previousYawRadians = 0;
        previousRollRadians = 0;

        pitchVelocity = 0;
        yawVelocity = 0;
        rollVelocity = 0;

        pitchAFilter.reset();
        yawAFilter.reset();
        rollAFilter.reset();
    }

    public void setSimState(
            double pitch, double yaw, double roll, double xAccel, double yAccel, double zAccel) {
        if (!Robot.isReal()) {
            simPitchRadians = pitch;
            simYawRadians = yaw;
            simRollRadians = roll;

            simXAccel = xAccel;
            simYAccel = yAccel;
            simZAccel = zAccel;
        } else {
            HighLevelLogger.getInstance()
                    .logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.getInstance()
                    .logWarning("There might be sim code still running somewhere!");
        }
    }

    public double getPitchRadians() {
        // WHY DOES THE NAVX DO THIS
        return Robot.isReal() ? headingDegreesToAngleRadians(navX.getRoll()) : simPitchRadians;
    }

    public double getPitchAngularVelocityRadiansPerSecond() {
        return pitchVelocity;
    }

    public double getPitchAngularAccelerationRadiansPerSecondSquared() {
        return pitchAFilter.getCurrentOutput();
    }

    public double getYawRadians() {
        return Robot.isReal() ? headingDegreesToAngleRadians(navX.getYaw()) : simYawRadians;
    }

    public double getYawAngularVelocityRadiansPerSecond() {
        return yawVelocity;
    }

    public double getYawAngularAccelerationRadiansPerSecondSquared() {
        return yawAFilter.getCurrentOutput();
    }

    public double getRollRadians() {
        // WHY IS THE NAVX LIKE THIS
        return Robot.isReal() ? headingDegreesToAngleRadians(navX.getPitch()) : simRollRadians;
    }

    public double getRollAngularVelocityRadiansPerSecond() {
        return rollVelocity;
    }

    public double getRollAngularAccelerationRadiansPerSecondSquared() {
        return rollAFilter.getCurrentOutput();
    }

    public double getXAccelerationMetersPerSecondSquared() {
        return Robot.isReal() ? gToMetersPerSecondSquared(navX.getWorldLinearAccelX()) : simXAccel;
    }

    public double getYAccelerationMetersPerSecondSquared() {
        return Robot.isReal() ? gToMetersPerSecondSquared(navX.getWorldLinearAccelY()) : simYAccel;
    }

    public double getZAccelerationMetersPerSecondSquared() {
        return Robot.isReal() ? gToMetersPerSecondSquared(navX.getWorldLinearAccelZ()) : simZAccel;
    }

    private double headingDegreesToAngleRadians(double headingDegrees) {
        return Units.degreesToRadians(-headingDegrees);
    }

    private double gToMetersPerSecondSquared(double accelG) {
        return accelG * 9.81;
    }

    @Override
    public void periodic() {
        isCalibratingLogger.update(isCalibrating());
        hasCalibratedLogger.update(hasCalibrated());

        pitchVelocity =
                UtilityMath.smallestAngleRadiansBetween(previousPitchRadians, getPitchRadians())
                        / 0.02;

        yawVelocity =
                UtilityMath.smallestAngleDegreesBetween(previousYawRadians, getYawRadians()) / 0.02;

        rollVelocity =
                UtilityMath.smallestAngleDegreesBetween(previousRollRadians, getRollRadians())
                        / 0.02;

        pitchLogger.update(getPitchRadians());
        yawLogger.update(getYawRadians());
        rollLogger.update(getRollRadians());

        pitchVLogger.update(pitchVelocity);
        yawVLogger.update(yawVelocity);
        rollVLogger.update(rollVelocity);

        pitchALogger.update(pitchAFilter.calculate(pitchVelocity));
        yawALogger.update(yawAFilter.calculate(yawVelocity));
        rollALogger.update(rollAFilter.calculate(rollVelocity));

        xALogger.update(getXAccelerationMetersPerSecondSquared());
        yALogger.update(getYAccelerationMetersPerSecondSquared());
        zALogger.update(getZAccelerationMetersPerSecondSquared());

        temperatureLogger.update(Robot.isReal() ? (double) navX.getTempC() : 0);
    }
}
