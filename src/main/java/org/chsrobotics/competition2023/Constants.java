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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import org.chsrobotics.lib.controllers.feedback.PID.PIDConstants;
import org.chsrobotics.lib.models.DoubleJointedArmModel;
import org.chsrobotics.lib.util.GearRatioHelper;
import org.photonvision.PhotonPoseEstimator;

public final class Constants {
    public static final class GLOBAL {
        public static final double GLOBAL_NOMINAL_VOLTAGE_VOLTS = 12;

        public static final double THERMAL_WARNING_THRESHOLD_C = 35;
        public static final double CURRENT_WARNING_THRESHOLD_A = 30;
    }

    public static final class SUBSYSTEM {
        public static final class DRIVETRAIN {
            public static final int LEFT_ENCODER_CHANNEL_A = 2;
            public static final int LEFT_ENCODER_CHANNEL_B = 3;
            public static final int RIGHT_ENCODER_CHANNEL_A = 0;
            public static final int RIGHT_ENCODER_CHANNEL_B = 1;

            public static final boolean LEFT_ENCODER_INVERTED = false;
            public static final boolean RIGHT_ENCODER_INVERTED = false;

            public static final GearRatioHelper ENCODER_TO_OUTPUT = new GearRatioHelper(1, 3);

            public static final int FRONT_RIGHT_CAN_ID = 4;
            public static final int BACK_RIGHT_CAN_ID = 5;
            public static final int FRONT_LEFT_CAN_ID = 2;
            public static final int BACK_LEFT_CAN_ID = 3;

            public static final boolean FRONT_RIGHT_IS_INVERTED = false;
            public static final boolean BACK_RIGHT_IS_INVERTED = false;
            public static final boolean FRONT_LEFT_IS_INVERTED = true;
            public static final boolean BACK_LEFT_IS_INVERTED = true;

            public static final boolean SHIFTER_SOLENOID_INVERTED = true;

            public static final int SHIFTER_SOLENOID_CHANNEL = 0;

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

            public static final String AT_CAMERA_FRONT_NAME = "pv_front";
            public static final String AT_CAMERA_BACK_NAME = "pv_back";

            public static final double AT_CAMERAS_DIAG_FOV_DEGREES = 180;

            public static final int AT_CAMERAS_HORIZONTAL_RESOLUTION_PX = 720;
            public static final int AT_CAMERAS_VERTICAL_RESOLUTION_PX = 540;

            public static final Transform3d ROBOT_TO_AT_FRONT_CAMERA =
                    new Transform3d(new Translation3d(0.262, -0.146, 0.257), new Rotation3d());

            public static final Transform3d ROBOT_TO_AT_BACK_CAMERA =
                    new Transform3d(
                            new Translation3d(-0.339, 0.2, 0.311), new Rotation3d(0, 0, Math.PI));

            public static final double AT_CAMERAS_SIMULATION_MIN_TARGET_AREA = 20;
        }

        public static final class CLAW {
            public static final int SOLENOID_CHANNEL = 14;

            public static final boolean SOLENOID_INVERTED = false;

            public static final int MOTOR_CAN_ID = 9;

            public static final boolean MOTOR_INVERTED = true;

            public static final double CURRENT_LIMIT_AMPS = 20;
        }

        public static final class POWER_DISTRIBUTION_HUB {}

        public static final class PNEUMATICS {
            public static final int ANALOG_PRESSURE_SENSOR_CHANNEL = 0;

            public static final double MIN_COMPRESSOR_PRESSURE_PSI = 100;
            public static final double MAX_PRESSURE_PSI = 120;

            public static final double PRESSURE_WARNING_THRESHOLD = 60;
        }

        public static final class INTAKE {
            public static final int LEFT_MOTOR_CANID = 9;
            public static final int RIGHT_MOTOR_CANID = 10;

            public static final boolean LEFT_MOTOR_INVERTED = false;
            public static final boolean RIGHT_MOTOR_INVERTED = false;

            public static final int DEPLOY_SOLENOID_CHANNEL = 15;
            public static final boolean DEPLOY_SOLENOID_INVERTED = false;

            public static final IdleMode IDLE_MODE = IdleMode.kCoast;

            public static final double DEPLOY_RUN_MOTORS_TIME_BUFFER_SECONDS = 0.5;

            public static final int CURRENT_LIMIT_AMPS = 20;
        }

        public static final class ARM {
            public static final int REVERSE_LIMIT_SWITCH_CHANNEL = 4;

            public static final boolean REVERSE_LIMIT_SWITCH_INVERTED = true;

            public static final double LOCAL_MIN_ANGLE = 0;
            public static final double LOCAL_MAX_ANGLE = 0;

            public static final double DISTAL_MIN_ANGLE = 0;
            public static final double DISTAL_MAX_ANGLE = 0;

            public static final int DISTAL_NEO_CAN_ID = 6;

            public static final int LEFT_LOCAL_NEO_CAN_ID = 7;

            public static final int RIGHT_LOCAL_NEO_CAN_ID = 8;

            public static final int DISTAL_POTENTIOMETER_ANALOG_CHANNEL = 1;

            public static final int LOCAL_POTENTIOMETER_ANALOG_CHANNEL = 0;

            public static final double DISTAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO = 3.70;

            public static final double LOCAL_POTENTIOMETER_REPORTED_ANGLE_RADIANS_AT_ZERO = 6.07;

            public static final double POTENTIOMETER_RANGE_RADIANS = 6 * Math.PI;

            public static final GearRatioHelper DISTAL_POTENTIOMETER_CONVERSION_HELPER =
                    new GearRatioHelper(1, 2);

            public static final GearRatioHelper LOCAL_POTENTIOMETER_CONVERSION_HELPER =
                    new GearRatioHelper(1, 2);

            public static final GearRatioHelper DISTAL_MOTOR_CONVERSION_HELPER =
                    new GearRatioHelper(1, 200);

            public static final GearRatioHelper LOCAL_MOTORS_CONVERSION_HELPER =
                    new GearRatioHelper(1, 200);

            public static final double LOCAL_ANGLE_SMOOTHING_RESPONSE_CONSTANT = 0.05;

            public static final double DISTAL_ANGLE_SMOOTHING_RESPONSE_CONSTANT = 0.05;

            public static final boolean DISTAL_NEO_INVERTED = false;

            public static final boolean LEFT_LOCAL_NEO_INVERTED = false;

            public static final boolean RIGHT_LOCAL_NEO_INVERTED = true;

            public static final double LOCAL_COM_POSITION_FROM_ROOT_METERS = 0.26;

            public static final double LOCAL_MASS_KG = 3.16;

            public static final double LOCAL_LENGTH_METERS = 0.86;

            public static final double LOCAL_MOMENT_ABOUT_COM = 1.91;

            public static final double DISTAL_COM_POSITION_FROM_ROOT_METERS = 0.61;

            public static final double DISTAL_MASS_KG = 1.9;

            public static final double DISTAL_LENGTH_METERS = 0.95;

            public static final double DISTAL_MOMENT_ABOUT_COM = 0.65;

            public static final IdleMode IDLE_MODE = IdleMode.kCoast;

            public static final Translation3d ROBOT_TO_ARM = new Translation3d(0.225, 0, 0.21);

            public static final double KG_SCALING = 0.1;

            public static final DoubleJointedArmModel ARM_MODEL =
                    new DoubleJointedArmModel(
                            LOCAL_MASS_KG,
                            LOCAL_COM_POSITION_FROM_ROOT_METERS,
                            LOCAL_MOMENT_ABOUT_COM,
                            LOCAL_LENGTH_METERS,
                            DCMotor.getNEO(2)
                                    .withReduction(
                                            LOCAL_MOTORS_CONVERSION_HELPER
                                                    .toDoubleRatioOutputToInput()),
                            DISTAL_MASS_KG,
                            DISTAL_COM_POSITION_FROM_ROOT_METERS,
                            DISTAL_MOMENT_ABOUT_COM,
                            DCMotor.getNEO(1)
                                    .withReduction(
                                            DISTAL_MOTOR_CONVERSION_HELPER
                                                    .toDoubleRatioOutputToInput()),
                            KG_SCALING);
        }
    }

    public static final class COMMAND {
        public static final class ARM_SETPOINT {
            public static final PIDConstants LOCAL_CONTROLLER_CONSTANTS = new PIDConstants(6, 0, 0);

            public static final int LOCAL_CONTROLLER_INTEGRATION_WINDOW = 100;

            public static final PIDConstants DISTAL_CONTROLLER_CONSTANTS =
                    new PIDConstants(5, 0, 0);

            public static final int DISTAL_CONTROLLER_INTEGRATION_WINDOW = 100;
        }

        public static final class ARM_JACOBIAN_CONTROL {
            public static final double INPUT_SCALING_FULL = 3;
            public static final double INPUT_SCALING_SLOW = 1;
        }

        public static final class ARM_NAVIGATE {
            public static final double ARM_NAVIGATE_TIME_SCALING = 0.5;

            public static final double SPLINE_TENSION = 0.4;
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

            public static final double MIN_SPEED = 0.3;
        }

        public static final class AUTO_BALANCE {
            public static final double KP = 6;
            public static final double KI = 0;
            public static final double KD = 0;
        }

        public static final class GO_OVER_RAMP {
            public static final double BALANCED_TOLERANCE_DEGREES = 2.5;

            public static final double FALLING_RATE_TOLERANCE_DEGREES_PER_SECOND = 5;

            public static final double CONTROL_INPUT_VOLTS = -9;
        }

        public static final class INTAKE_COMMAND {
            public static final double OPEN_LOOP_VOLTAGE = 7.5;
        }
    }
}
