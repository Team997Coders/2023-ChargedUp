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

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.InertialMeasurement;
import org.chsrobotics.competition2023.subsystems.Vision;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.telemetry.Logger;

public class Localizer {
    private static final Localizer instance = new Localizer();

    private final Vision vision = Vision.getInstance();

    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final InertialMeasurement imu = InertialMeasurement.getInstance();

    private final Field2d field2d = new Field2d();

    private final DifferentialDrivePoseEstimator poseEstimator =
            new DifferentialDrivePoseEstimator(
                    new DifferentialDriveKinematics(1), new Rotation2d(), 0, 0, new Pose2d());

    private final String subdirString = "localizer";

    private final Logger<Double[]> poseLogger = new Logger<>("pose_m_m_rad", subdirString);

    private Localizer() {
        HighLevelLogger.getInstance().publishSendable("RobotPose", field2d);
    }

    public static Localizer getInstance() {
        return instance;
    }

    public void periodic() {
        field2d.setRobotPose(getEstimatedPose());

        poseLogger.update(
                new Double[] {
                    getEstimatedPose().getX(),
                    getEstimatedPose().getY(),
                    getEstimatedPose().getRotation().getRadians()
                });

        poseEstimator.updateWithTime(
                Units.millisecondsToSeconds(System.currentTimeMillis()),
                new Rotation2d(InertialMeasurement.getInstance().getYawRadians()),
                Drivetrain.getInstance().getLeftSensorPosition(),
                Drivetrain.getInstance().getRightSensorPosition());
                
        if (vision.getCurrentPoseEstimates() != null) {
            var estimates = vision.getCurrentPoseEstimates();
            if (estimates.firstValue() != null) {
                var measurement = estimates.firstValue();
                poseEstimator.addVisionMeasurement(
                        measurement.estimatedPose.toPose2d(), measurement.timestampSeconds);
            }
            if (estimates.secondValue() != null) {
                var measurement = estimates.secondValue();
                poseEstimator.addVisionMeasurement(
                        measurement.estimatedPose.toPose2d(), measurement.timestampSeconds);
            }
        }
    }

    public void setPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                new Rotation2d(InertialMeasurement.getInstance().getYawRadians()),
                Drivetrain.getInstance().getLeftSensorPosition(),
                Drivetrain.getInstance().getRightSensorPosition(),
                newPose);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
