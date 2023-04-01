// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class Ports {
		public static final int PIGEON_2 = 4;

		public static final int CLAW_LEFT_WHEELS = 13;
		public static final int CLAW_RIGHT_WHEELS = 12;

		public static final int ELEVATOR_A = 9;
		public static final int ELEVATOR_B = 10;

		public static final int WRIST = 11;

		public static final int CLAW = 12;

		public static final int BUDDYCLIMB_WINCH_A = 14;
		public static final int BUDDYCLIMB_WINCH_B = 15;

		public static final int CLAW_SOLENOID = 8;

		// public static final int BUDDYCLIMB_LIFT_A = 10;
		// public static final int BUDDYCLIMB_LIFT_B = 7;
		// public static final int BUDDYCLIMB_DEPLOY_A = 11;
		// public static final int BUDDYCLIMB_DEPLOY_B = 9;

		// public static final int ELEVATOR_LIMIT_SWITCH = 1;

		public static final int PH_CAN_ID = 10;

		public static final int LED_OUTPUT_A = 0;
		public static final int LED_OUTPUT_B = 1;
		public static final int LED_OUTPUT_C = 2;
	}

	public static final class Elevator {
		// Elevator PID //
		public static final double ELEVATOR_KP = 0.05;
		public static final double ELEVATOR_KI = 0;
		public static final double ELEVATOR_KD = 0;

		// Elevator FF //
		public static final double ELEVATOR_KS = 0.032;
		public static final double ELEVATOR_KG = 0;
		public static final double ELEVATOR_KV = 0;
		public static final double ELEVATOR_MAX_VEL = 0;
		public static final double ELEVATOR_MAX_ACCEL = 0;
	}

	public static final class Wrist {
		public static final double WRIST_KP = 0.008;
		public static final double WRIST_KI = 0;
		public static final double WRIST_KD = 0;
		public static final double WRIST_KS = 0.025;
		public static final double WRIST_KG = 0.025;
		public static final double WRIST_KV = 0;

		public static final double WRIST_GEAR_RATIO = 127.27272727;
		public static final double WRIST_MAX_VEL = 0;
		public static final double WRIST_MAX_ACCEL = 0;
	}

	public static final class Vision {
		public static final Transform3d cameraToRobot = new Transform3d(
				new Translation3d(Units.inchesToMeters(13.25), 0, 0),
				new Rotation3d(0, Units.degreesToRadians(-20), 0));

		public static final double X_P = 1.0;
		public static final double X_I = 0.0;
		public static final double X_D = 0;

		public static final double Y_P = 1.0;
		public static final double Y_I = 0.0;
		public static final double Y_D = 0;

		public static final double THETA_P = .8;
		public static final double THETA_I = 0.0;
		public static final double THETA_D = 0;

		public static final double X_TOLLERENCE = 0.01;
		public static final double Y_TOLLERENCE = 0.02;
		public static final double THETA_TOLLERENCE = 0.02;
	}

	public static final class Swerve {

		/* Drive Feedforward */
		public static final double DRIVE_KS = 0.11937 / 12;
		public static final double DRIVE_KV = 2.6335 / 12;
		public static final double DRIVE_KA = 0.46034 / 12;

		/* Swerve Gear Ratios */
		public static final double DRIVE_GEAR_RATIO = (6.12 / 1); // 6.12 : 1
		public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1); // 150/7 : 1

		/* Swerve Inverts */
		public static final boolean DRIVE_MOTOR_INVERT = false;
		public static final boolean ANGLE_MOTOR_INVERT = true;
		public static final boolean INVERT_GYRO = false;
		public static final boolean CANCONDER_INVERT = false;

		/* Kinematics */
		public static final double TRACK_WIDTH = Units.inchesToMeters(21.70);
		public static final double WHEEL_BASE = Units.inchesToMeters(21.70);
		public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

		public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

		/* Ramp Rates and Current Limits */
		public static final double DRIVE_CLOSED_LOOP_RAMP = 0;
		public static final double DRIVE_OPEN_LOOP_RAMP = 0;
		public static final int ANGLE_MOTOR_SMART_CURRENT = 25;
		public static final double ANGLE_MOTOR_SECONDARY_LIMIT = 40;
		public static final int DRIVE_MOTOR_SMART_CURRENT = 35;
		public static final double DRIVE_MOTOR_SECONDARY_LIMIT = 60;

		/* Angle Motor PID */
		public static final double ANGLE_MOTOR_KP = 0.005;
		public static final double ANGLE_MOTOR_KI = 0;
		public static final double ANGLE_MOTOR_KD = 0.0000;

		/* Drive Motor PID */
		public static final double DRIVE_MOTOR_KP = 0.1;
		public static final double DRIVE_MOTOR_KI = 0;
		public static final double DRIVE_MOTOR_KD = 0;
		public static final double DRIVE_MOTOR_KF = 0;

		public static final double DRIVE_MOTOR_MIN_OUTPUT = -1;
		public static final double DRIVE_MOTOR_MAX_OUTPUT = 1;

		/* Conversion Factors */
		public static final double DRIVE_MOTOR_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
		public static final double DRIVE_MOTOR_VELOCITY_CONVERSION = (WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO) / 60;
		public static final double ANGLE_MOTOR_POSITION_CONVERSION = 360 / ANGLE_GEAR_RATIO;
		public static final double ANGLE_MOTOR_VELOCITY_CONVERSION = (360 / ANGLE_GEAR_RATIO) / 60;

		public static final SwerveModuleState[] ZERO_STATES = {
				new SwerveModuleState(),
				new SwerveModuleState(),
				new SwerveModuleState(),
				new SwerveModuleState()
		};

		/* Max Speed */
		public static final double MAX_ANGULAR_VELOCITY = 11.5;
		public static final double MAX_SPEED = 5.5;

		/* Module Constants */
		// BEVEL GEARS FACE RADIO FOR OFFSETS
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final int driveMotorID = 1;
			public static final int angleMotorID = 2;
			public static final int canCoderID = 0;
			public static final double angleOffset = 185.09765625;
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
					angleMotorID,
					canCoderID, angleOffset);
		}

		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 3;
			public static final int angleMotorID = 4;
			public static final int canCoderID = 1;
			public static final double angleOffset = 201.884765625;
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
					angleMotorID,
					canCoderID, angleOffset);
		}

		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 5;
			public static final int angleMotorID = 6;
			public static final int canCoderID = 2;
			public static final double angleOffset = 69.78515625;
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
					angleMotorID,
					canCoderID, angleOffset);
		}

		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 7;
			public static final int angleMotorID = 8;
			public static final int canCoderID = 3;
			public static final double angleOffset = 225.263671875;
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
					angleMotorID,
					canCoderID, angleOffset);
		}
	}

	public static final class AutoConstants {
		public static final double K_MAX_SPEED_METERS_PER_SECOND = 4;
		public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5;
		public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
		public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

		public static final double KPX_CONTROLLER = 1;
		public static final double KPY_CONTROLLER = 1;
		public static final double KP_THETA_CONTROLLER = 1;

		// Constraint for the motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
				K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
	}
}