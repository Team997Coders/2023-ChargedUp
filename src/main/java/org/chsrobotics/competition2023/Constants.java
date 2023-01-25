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
import org.chsrobotics.lib.util.GearRatioHelper;
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
        public static final class DRIVETRAIN {
            public static final int FRONT_RIGHT_CAN_ID = 2;
            public static final int BACK_RIGHT_CAN_ID = 4;
            public static final int FRONT_LEFT_CAN_ID = 1;
            public static final int BACK_LEFT_CAN_ID = 3;

            public static final boolean FRONT_RIGHT_IS_INVERTED = false;
            public static final boolean BACK_RIGHT_IS_INVERTED = false;
            public static final boolean FRONT_LEFT_IS_INVERTED = false;
            public static final boolean BACK_LEFT_IS_INVERTED = false;

            public static final boolean LEFT_SHIFTER_SOLENOID_IS_INVERTED = false;
            public static final boolean RIGHT_SHIFTER_SOLENOID_IS_INVERTED = false;

            public static final int LEFT_SHIFTER_SOLENOID_CHANNEL = 0;
            public static final int RIGHT_SHIFTER_SOLENOID_CHANNEL = 0;

            public static final GearRatioHelper SLOW_GEAR_RATIO = new GearRatioHelper(1, 1);

            public static final GearRatioHelper FAST_GEAR_RATIO = new GearRatioHelper(2, 1);

            public static final double WHEEL_RADIUS_METERS = 0;
        }

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

        public static final class INERTIAL_MEASUREMENT {}

        public static final class POWER_DISTRIBUTION_HUB {}

        public static final class ARM {
            public static final int DISTAL_NEO_CAN_ID = 0;

            public static final int LOCAL_NEO_A_CAN_ID = 0;

            public static final int LOCAL_NEO_B_CAN_ID = 0;

            public static final int DISTAL_POTENTIOMETER_ANALOG_CHANNEL = 1;

            public static final int LOCAL_POTENTIOMETER_ANALOG_CHANNEL = 2;

            public static final int DISTAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO = 0;

            public static final int LOCAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO = 0;

            public static final GearRatioHelper DISTAL_POTENTIOMTER_CONVERSION_HELPER =
                    new GearRatioHelper(1, 1);

            public static final GearRatioHelper LOCAL_POTENTIOMETER_CONVERSION_HELPER =
                    new GearRatioHelper(1, 1);

            public static final boolean DISTAL_NEO_INVERTED = false;

            public static final boolean LOCAL_NEO_A_INVERTED = false;

            public static final boolean LOCAL_NEO_B_INVERTED = false;
        }
    }

    public static final class COMMAND {
        public static final class TELEOP_DRIVE {
            public static final double ACCELERATION_LIMITER_TRACKWIDTH = 0;
            public static final double ACCELERATION_LIMITER_MAX_LINEAR_ACCEL = 0;
            public static final double ACCELERATION_LIMITER_MAX_ANGULAR_ACCEL = 0;

            public static final double FEEDFORWARD_KVLINEAR = 0;
            public static final double FEEDFORWARD_KALINEAR = 0;
            public static final double FEEDFORWARD_KVANGULAR = 0;
            public static final double FEEDFORWARD_KAANGULAR = 0;
            public static final double FEEDFORWARD_TRACKWIDTH = 0;
        }
    }
}
