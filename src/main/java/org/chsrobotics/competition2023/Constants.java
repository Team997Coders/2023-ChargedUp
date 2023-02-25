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

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import org.chsrobotics.competition2023.util.CSpacePackageLoader;
import org.chsrobotics.competition2023.util.CSpacePackageLoader.CSpacePackage;
import org.chsrobotics.lib.controllers.feedback.PID.PIDConstants;
import org.chsrobotics.lib.util.GearRatioHelper;
import org.photonvision.PhotonPoseEstimator;

public final class Constants {
    public static final class GLOBAL {
        public static final double GLOBAL_NOMINAL_VOLTAGE_VOLTS = 12;
    }

    public static final class SUBSYSTEM {
        public static final class DRIVETRAIN {
            public static final int LEFT_ENCODER_CHANNEL_A = 0;
            public static final int LEFT_ENCODER_CHANNEL_B = 1;
            public static final int RIGHT_ENCODER_CHANNEL_A = 2;
            public static final int RIGHT_ENCODER_CHANNEL_B = 3;

            public static final boolean LEFT_ENCODER_INVERTED = false;
            public static final boolean RIGHT_ENCODER_INVERTED = false;

            public static final GearRatioHelper ENCODER_TO_OUTPUT = new GearRatioHelper(1, 3);

            public static final int FRONT_RIGHT_CAN_ID = 4;
            public static final int BACK_RIGHT_CAN_ID = 5;
            public static final int FRONT_LEFT_CAN_ID = 2;
            public static final int BACK_LEFT_CAN_ID = 3;

            public static final boolean FRONT_RIGHT_IS_INVERTED = false;
            public static final boolean BACK_RIGHT_IS_INVERTED = false;
            public static final boolean FRONT_LEFT_IS_INVERTED = false;
            public static final boolean BACK_LEFT_IS_INVERTED = false;

            public static final boolean SHIFTER_SOLENOID_INVERTED = false;

            public static final int SHIFTER_SOLENOID_CHANNEL = 3;

            public static final GearRatioHelper SLOW_GEAR_RATIO = new GearRatioHelper(1, 22.67);

            public static final GearRatioHelper FAST_GEAR_RATIO = new GearRatioHelper(1, 7.56);

            public static final double WHEEL_RADIUS_METERS = 0.152;

            public static final double TRACKWIDTH_METERS = 1;

            public static final double SLOW_KV_LINEAR = 1;
            public static final double SLOW_KA_LINEAR = 1;

            public static final double SLOW_KV_ANGULAR = 1;
            public static final double SLOW_KA_ANGULAR = 1;

            public static final LinearSystem<N2, N2, N2> SLOW_DRIVETRAIN_PLANT =
                    LinearSystemId.identifyDrivetrainSystem(
                            SLOW_KV_LINEAR,
                            SLOW_KA_LINEAR,
                            SLOW_KV_ANGULAR,
                            SLOW_KA_ANGULAR,
                            TRACKWIDTH_METERS);

            public static final double FAST_KV_LINEAR = 1;
            public static final double FAST_KA_LINEAR = 1;

            public static final double FAST_KV_ANGULAR = 1;
            public static final double FAST_KA_ANGULAR = 1;

            public static final LinearSystem<N2, N2, N2> FAST_DRIVETRAIN_PLANT =
                    LinearSystemId.identifyDrivetrainSystem(
                            FAST_KV_LINEAR,
                            FAST_KA_LINEAR,
                            FAST_KV_ANGULAR,
                            FAST_KA_ANGULAR,
                            TRACKWIDTH_METERS);
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
                    new Transform3d(new Translation3d(0.262, -0.146, 0.257), new Rotation3d());

            public static final Transform3d ROBOT_TO_AT_CAMERA_B =
                    new Transform3d(
                            new Translation3d(-0.039, -0.034, 0.311),
                            new Rotation3d(0, 0, Math.PI));

            public static final double AT_CAMERAS_SIMULATION_MIN_TARGET_AREA = 20;
        }

        public static final class INTAKE {
            public static final int LEFT_MOTOR_CANID = 9;
            public static final int RIGHT_MOTOR_CANID = 10;

            public static final int DEPLOY_SOLENOID_CHANNEL = 2;

            public static final boolean LEFT_MOTOR_INVERTED = false;
            public static final boolean RIGHT_MOTOR_INVERTED = false;

            public static final boolean DEPLOY_SOLENOID_INVERTED = false;

            public static final double DEPLOY_RUN_MOTORS_TIME_BUFFER_SECONDS = 0.1;

            public static final IdleMode IDLE_MODE = IdleMode.kCoast;
        }

        public static final class GRABBER {
            public static final int SOLENOID_CHANNEL = 1;

            public static final boolean SOLENOID_INVERTED = false;
        }

        public static final class INERTIAL_MEASUREMENT {}

        public static final class POWER_DISTRIBUTION_HUB {}

        public static final class ARM {
            public static final int DISTAL_NEO_CAN_ID = 6;

            public static final int LEFT_LOCAL_NEO_CAN_ID = 7;

            public static final int RIGHT_LOCAL_NEO_CAN_ID = 8;

            public static final int DISTAL_POTENTIOMETER_ANALOG_CHANNEL = 1;

            public static final int LOCAL_POTENTIOMETER_ANALOG_CHANNEL = 2;

            public static final int DISTAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO = 0;

            public static final int LOCAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO = 0;

            public static final GearRatioHelper DISTAL_POTENTIOMETER_CONVERSION_HELPER =
                    new GearRatioHelper(1, 2);

            public static final GearRatioHelper LOCAL_POTENTIOMETER_CONVERSION_HELPER =
                    new GearRatioHelper(1, 2);

            public static final GearRatioHelper DISTAL_MOTOR_CONVERSION_HELPER =
                    new GearRatioHelper(1, 200);

            public static final GearRatioHelper LOCAL_MOTORS_CONVERSION_HELPER =
                    new GearRatioHelper(1, 200);

            public static final double LOCAL_ANGLE_SMOOTHING_RESPONSE_CONSTANT = 0.25;

            public static final double DISTAL_ANGLE_SMOOTHING_RESPONSE_CONSTANT = 0.25;

            public static final boolean DISTAL_NEO_INVERTED = false;

            public static final boolean LEFT_LOCAL_NEO_INVERTED = false;

            public static final boolean RIGHT_LOCAL_NEO_INVERTED = false;

            public static final double LOCAL_COM_POSITION_FROM_ROOT_METERS = 0.26;

            public static final double LOCAL_MASS_KG = 3.16;

            public static final double LOCAL_LENGTH_METERS = 0.86;

            public static final double LOCAL_MOMENT_ABOUT_COM = 1.91;

            public static final double DISTAL_COM_POSITION_FROM_ROOT_METERS = 0.45;

            public static final double DISTAL_MASS_KG = 1.9;

            public static final double DISTAL_LENGTH_METERS = 0.81;

            public static final double DISTAL_MOMENT_ABOUT_COM = 1.65;

            public static final IdleMode IDLE_MODE = IdleMode.kBrake;

            public static final Translation3d ROBOT_TO_ARM = new Translation3d(0.225, 0, 0.21);
        }
    }

    public static final class COMMAND {
        public static final class ARM_SETPOINT {
            public static final PIDConstants LOCAL_CONTROLLER_CONSTANTS =
                    new PIDConstants(24, 0, 0);

            public static final int LOCAL_CONTROLLER_INTEGRATION_WINDOW = -1;

            public static final PIDConstants DISTAL_CONTROLLER_CONSTANTS =
                    new PIDConstants(36, 0, 0);

            public static final int DISTAL_CONTROLLER_INTEGRATION_WINDOW = -1;
        }

        public static final class ARM_CARTESIAN_CONTROL {
            public static final double MAX_SETPOINT_VELOCITY_METERS_PER_SECOND = 1;
        }

        public static final class ARM_NAVIGATE {
            public static final double ARM_NAVIGATE_TIME_SCALING = 1;

            public static final double SPLINE_TENSION = 0.4;

            public static final CSpacePackage FREE_NO_INTAKE =
                    CSpacePackageLoader.loadPackage(
                            new File(Filesystem.getDeployDirectory(), "/cspace/freeNoIntake.json"));
        }

        public static final class TRAJECTORY_FOLLOWING {
            public static final double K_MAX_SPEED_METERS_PER_SECOND = 1.0;
            public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.0;

            public static final double K_RAMSETE_B = 2.2;
            public static final double K_RAMSETE_ZETA = 0.7;
        }

        public static final class TELEOP_DRIVE {
            public static final double FAST_MAX_LINEAR_ACCEL_M_P_SEC_SQUARED = 5;
            public static final double FAST_MAX_ANGULAR_ACCEL_RADS_P_SEC_SQUARED = 5;

            public static final double SLOW_MAX_LINEAR_ACCEL_M_P_SEC_SQUARED = 3;

            public static final double SLOW_MAX_ANGULAR_ACCEL_RAD_P_SEC_SQUARED = 3;
        }
    }
}
