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

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public final class Constants {
    public static final class GLOBAL {
        public static final double GLOBAL_NOMINAL_VOLTAGE_VOLTS = 12;

        public static final AprilTagFieldLayout TAG_LAYOUT =
                new AprilTagFieldLayout(List.of(new AprilTag(0, new Pose3d())), 0, 0);

        public static final double APRILTAG_WIDTH_METERS = 1;
        public static final double APRILTAG_HEIGHT_METERS = 1;
    }

    public static final class SUBSYSTEM {
        public static final class VISION {
            public static final PoseStrategy DEFAULT_POSE_STRATEGY =
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE;

            public static final String AT_CAMERA_A_NAME = "CameraA";
            public static final String AT_CAMERA_B_NAME = "CameraB";

            public static final double AT_CAMERAS_DIAG_FOV_DEGREES = 180;

            public static final int AT_CAMERAS_HORIZONTAL_RESOLUTION_PX = 720;
            public static final int AT_CAMERAS_VERTICAL_RESOLUTION_PX = 540;

            public static final Transform3d ROBOT_TO_AT_CAMERA_A =
                    new Transform3d(new Translation3d(1, 1, 1), new Rotation3d());

            public static final Transform3d ROBOT_TO_AT_CAMERA_B =
                    new Transform3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, Math.PI));

            public static final double AT_CAMERAS_SIMULATION_MIN_TARGET_AREA = 20;
        }

        public static final class DRIVETRAIN {}

        public static final class INTAKE {
            public static final int LEFT_MOTOR_CANID = 0;
            public static final int RIGHT_MOTOR_CANID = 0;

            public static final int LEFT_DEPLOY_SOLENOID_CHANNEL = 0;
            public static final int RIGHT_DEPLOY_SOLENOID_CHANNEL = 0;

            public static final boolean LEFT_MOTOR_INVERTED = false;
            public static final boolean RIGHT_MOTOR_INVERTED = false;

            public static final boolean LEFT_DEPLOY_SOLENOID_INVERTED = false;
            public static final boolean RIGHT_DEPLOY_SOLENOID_INVERTED = false;

            public static final double DEPLOY_RUN_MOTORS_TIME_BUFFER_SECONDS = 0.1;
        }

        public static final class INERTIAL_MEASUREMENT {}

        public static final class POWER_DISTRIBUTION_HUB {}
    }
}
