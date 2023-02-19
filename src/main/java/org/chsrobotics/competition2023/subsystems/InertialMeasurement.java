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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Robot;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.math.filters.IntegratingFilter;
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

    private final DifferentiatingFilter pitchJFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter yawJFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter rollJFilter = new DifferentiatingFilter();

    private final DifferentiatingFilter xJFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter yJFilter = new DifferentiatingFilter();
    private final DifferentiatingFilter zJFilter = new DifferentiatingFilter();

    private final IntegratingFilter xVFilter = new IntegratingFilter(0);
    private final IntegratingFilter yVFilter = new IntegratingFilter(0);
    private final IntegratingFilter zVFilter = new IntegratingFilter(0);

    private final IntegratingFilter xFilter = new IntegratingFilter(0);
    private final IntegratingFilter yFilter = new IntegratingFilter(0);
    private final IntegratingFilter zFilter = new IntegratingFilter(0);

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

    private final Logger<Double> pitchJLogger = new Logger<>("pitchJerk_rad_p_s^3", subdirString);
    private final Logger<Double> yawJLogger = new Logger<>("yawJerk_rad_p_s^3", subdirString);
    private final Logger<Double> rollJLogger = new Logger<>("rollJerk_rad_p_s^3", subdirString);

    private final Logger<Double> xALogger = new Logger<>("xAcceleration_m_p_s^2", subdirString);
    private final Logger<Double> yALogger = new Logger<>("yAcceleration_m_p_s^2", subdirString);
    private final Logger<Double> zALogger = new Logger<>("zAcceleration_m_p_s^2", subdirString);

    private final Logger<Double> xJLogger = new Logger<>("xJerk_m_p_s^3", subdirString);
    private final Logger<Double> yJLogger = new Logger<>("yJerk_m_p_s^3", subdirString);
    private final Logger<Double> zJLogger = new Logger<>("zJerk_m_p_s^3", subdirString);

    private final Logger<Double> xVLogger = new Logger<>("xVelocity_m_p_s", subdirString);
    private final Logger<Double> yVLogger = new Logger<>("yVelocity_m_p_s", subdirString);
    private final Logger<Double> zVLogger = new Logger<>("zVelocity_m_p_s", subdirString);

    private final Logger<Double> xLogger = new Logger<>("x_m", subdirString);
    private final Logger<Double> yLogger = new Logger<>("y_m", subdirString);
    private final Logger<Double> zLogger = new Logger<>("z_m", subdirString);

    private final Logger<Double> temperatureLogger = new Logger<>("temperature_C", subdirString);

    private double simPitchRadians = 0;
    private double simYawRadians = 0;
    private double simRollRadians = 0;

    private double simXAccel = 0;
    private double simYAccel = 0;
    private double simZAccel = 0;

    private Rotation3d rotation = new Rotation3d();

    private Translation3d accel = new Translation3d();

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

        pitchJFilter.reset();
        yawJFilter.reset();
        rollJFilter.reset();

        xJFilter.reset();
        yJFilter.reset();
        zJFilter.reset();

        xVFilter.reset();
        yVFilter.reset();
        zVFilter.reset();

        xFilter.reset();
        yFilter.reset();
        zFilter.reset();
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
        return rotation.getY();
    }

    public double getPitchAngularVelocityRadiansPerSecond() {
        return pitchVelocity;
    }

    public double getPitchAngularAccelerationRadiansPerSecondSquared() {
        return pitchAFilter.getCurrentOutput();
    }

    public double getPitchAngularJerkRadiansPerSecondCubed() {
        return pitchJFilter.getCurrentOutput();
    }

    public double getYawRadians() {
        return rotation.getZ();
    }

    public double getYawAngularVelocityRadiansPerSecond() {
        return yawVelocity;
    }

    public double getYawAngularAccelerationRadiansPerSecondSquared() {
        return yawAFilter.getCurrentOutput();
    }

    public double getYawAngularJerkRadiansPerSecondCubed() {
        return yawJFilter.getCurrentOutput();
    }

    public double getRollRadians() {
        return rotation.getX();
    }

    public double getRollAngularVelocityRadiansPerSecond() {
        return rollVelocity;
    }

    public double getRollAngularAccelerationRadiansPerSecondSquared() {
        return rollAFilter.getCurrentOutput();
    }

    public double getRollAngularJerkRadiansPerSecondCubed() {
        return rollJFilter.getCurrentOutput();
    }

    public double getXAccelerationMetersPerSecondSquared() {
        return accel.getX();
    }

    public double getXJerkMetersPerSecondCubed() {
        return xJFilter.getCurrentOutput();
    }

    public double getRobotFrameXVelocityMetersPerSecond() {
        return xVFilter.getCurrentOutput();
    }

    public double getRobotFrameXMeters() {
        return xFilter.getCurrentOutput();
    }

    public double getYAccelerationMetersPerSecondSquared() {
        return accel.getY();
    }

    public double getYJerkMetersPerSecondCubed() {
        return yJFilter.getCurrentOutput();
    }

    public double getRobotFrameYVelocityMetersPerSecond() {
        return yVFilter.getCurrentOutput();
    }

    public double getRobotFrameYMeters() {
        return yFilter.getCurrentOutput();
    }

    public double getZAccelerationMetersPerSecondSquared() {
        return accel.getZ();
    }

    public double getZJerkMetersPerSecondCubed() {
        return zJFilter.getCurrentOutput();
    }

    public double getRobotFrameZVelocityMetersPerSecond() {
        return zVFilter.getCurrentOutput();
    }

    public double getRobotFrameZMeters() {
        return zFilter.getCurrentOutput();
    }

    private double headingDegreesToAngleRadians(double headingDegrees) {
        return Units.degreesToRadians(-headingDegrees);
    }

    private double gToMetersPerSecondSquared(double accelG) {
        return accelG * 9.81;
    }

    @Override
    public void periodic() {
        double relPitch =
                Robot.isReal() ? headingDegreesToAngleRadians(navX.getPitch()) : simPitchRadians;

        double relRoll =
                Robot.isReal() ? headingDegreesToAngleRadians(navX.getRoll()) : simRollRadians;

        double relYaw =
                Robot.isReal() ? headingDegreesToAngleRadians(navX.getYaw()) : simYawRadians;

        Rotation3d relRot = new Rotation3d(relRoll, relPitch, relYaw);

        rotation = relRot.plus(Constants.SUBSYSTEM.INERTIAL_MEASUREMENT.ROBOT_TO_NAVX);

        double relX = Robot.isReal() ? gToMetersPerSecondSquared(navX.getRawAccelX()) : simXAccel;

        double relY = Robot.isReal() ? gToMetersPerSecondSquared(navX.getRawAccelY()) : simYAccel;

        double relZ = Robot.isReal() ? gToMetersPerSecondSquared(navX.getRawAccelZ()) : simZAccel;

        Translation3d accelsAsTranslation = new Translation3d(relX, relY, relZ);

        accel =
                accelsAsTranslation.rotateBy(
                        Constants.SUBSYSTEM.INERTIAL_MEASUREMENT.ROBOT_TO_NAVX);

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

        pitchJLogger.update(pitchJFilter.calculate(pitchAFilter.getCurrentOutput()));
        yawJLogger.update(yawJFilter.calculate(yawAFilter.getCurrentOutput()));
        rollJLogger.update(rollJFilter.calculate(rollAFilter.getCurrentOutput()));

        xALogger.update(getXAccelerationMetersPerSecondSquared());
        yALogger.update(getYAccelerationMetersPerSecondSquared());
        zALogger.update(getZAccelerationMetersPerSecondSquared());

        xJLogger.update(xJFilter.calculate(getXAccelerationMetersPerSecondSquared()));
        yJLogger.update(yJFilter.calculate(getYAccelerationMetersPerSecondSquared()));
        zJLogger.update(zJFilter.calculate(getZAccelerationMetersPerSecondSquared()));

        xVLogger.update(xVFilter.calculate(getXAccelerationMetersPerSecondSquared()));
        yVLogger.update(yVFilter.calculate(getYAccelerationMetersPerSecondSquared()));
        zVLogger.update(zVFilter.calculate(getZAccelerationMetersPerSecondSquared()));

        xLogger.update(xFilter.calculate(xVFilter.getCurrentOutput()));
        yLogger.update(yFilter.calculate(yVFilter.getCurrentOutput()));
        zLogger.update(zFilter.calculate(zVFilter.getCurrentOutput()));

        temperatureLogger.update(Robot.isReal() ? (double) navX.getTempC() : 0);
    }
}
