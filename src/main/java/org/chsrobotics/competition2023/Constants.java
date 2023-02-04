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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.chsrobotics.lib.controllers.feedback.PID.PIDConstants;
import org.chsrobotics.lib.util.GearRatioHelper;
import org.photonvision.PhotonPoseEstimator;

public final class Constants {
    public static final class GLOBAL {
        public static final double GLOBAL_NOMINAL_VOLTAGE_VOLTS = 12;

        public static final double APRILTAG_WIDTH_METERS = 1;
        public static final double APRILTAG_HEIGHT_METERS = 1;
    }

    public static final class SUBSYSTEM {
        public static final class DRIVETRAIN {
            public static final int FRONT_RIGHT_CAN_ID = 0;
            public static final int BACK_RIGHT_CAN_ID = 0;
            public static final int FRONT_LEFT_CAN_ID = 0;
            public static final int BACK_LEFT_CAN_ID = 0;

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
            public static final PhotonPoseEstimator.PoseStrategy DEFAULT_POSE_STRATEGY =
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE;

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

        public static final class GRABBER {
            public static final int SOLENOID_CHANNEL = 0;

            public static final boolean SOLENOID_INVERTED = false;
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

            public static final GearRatioHelper LOCAL_NEO_TO_ARM_HELPER =
                    new GearRatioHelper(1, 200);

            public static final GearRatioHelper DISTAL_NEO_TO_ARM_HELPER =
                    new GearRatioHelper(1, 100);

            public static final boolean DISTAL_NEO_INVERTED = false;

            public static final boolean LOCAL_NEO_A_INVERTED = false;

            public static final boolean LOCAL_NEO_B_INVERTED = false;

            public static final double LOCAL_COM_POSITION_FROM_ROOT_METERS = 0.5;

            public static final double LOCAL_MASS_KG = 2;

            public static final double LOCAL_LENGTH_METERS = 1;

            public static final double LOCAL_MOMENT_ABOUT_COM = 2;

            public static final double DISTAL_COM_POSITION_FROM_ROOT_METERS = 0.5;

            public static final double DISTAL_MASS_KG = 2;

            public static final double DISTAL_LENGTH_METERS = 1;

            public static final double DISTAL_MOMENT_ABOUT_COM = 2;
        }
    }

    public static final class COMMAND {
        public static final class ARM_SETPOINT {
            public static final PIDConstants LOCAL_CONTROLLER_CONSTANTS = new PIDConstants(0, 0, 0);

            public static final int LOCAL_CONTROLLER_INTEGRATION_WINDOW = -1;

            public static final PIDConstants DISTAL_CONTROLLER_CONSTANTS =
                    new PIDConstants(0, 0, 0);

            public static final int DISTAL_CONTROLLER_INTEGRATION_WINDOW = -1;
        }

        public static final class ARM_CARTESIAN_CONTROL {
            public static final double MAX_SETPOINT_VELOCITY_METERS_PER_SECOND = 0.5;
        }
    }
}
