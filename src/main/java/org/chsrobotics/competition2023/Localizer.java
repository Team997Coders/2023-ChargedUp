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
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.competition2023.subsystems.InertialMeasurement;
import org.chsrobotics.competition2023.subsystems.Vision;

public class Localizer {
    private static final Localizer instance = new Localizer();

    private final Vision vision = Vision.getInstance();

    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final InertialMeasurement imu = InertialMeasurement.getInstance();

    private final DifferentialDrivePoseEstimator poseEstimator =
            new DifferentialDrivePoseEstimator(
                    new DifferentialDriveKinematics(1), // drivetrain trackwidth
                    new Rotation2d(),
                    0,
                    0,
                    new Pose2d());

    private Localizer() {}

    public static Localizer getInstance() {
        return instance;
    }

    public void periodic() {
        poseEstimator.updateWithTime(
                Units.millisecondsToSeconds(System.currentTimeMillis()),
                new Rotation2d(0), // imu yaw
                0, // drivetrain left encoder meters
                0); // drivetrain right encoder meters

        if (vision.getCurrentPose2dEstimate() != null) {
            var estimate = vision.getCurrentPose2dEstimate();
            poseEstimator.addVisionMeasurement(estimate.getFirst(), estimate.getSecond());
        }
    }

    public void setPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                new Rotation2d(0), // IMU yaw
                0, // drivetrain right encoder meters
                0, // drivetrain left encoder meters
                newPose);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
