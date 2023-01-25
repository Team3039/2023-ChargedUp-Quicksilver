// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Ports{ 
        public static final int PIGEON_2 = 0;

        public static final int CLAW_LEFT_WHEELS = 1;
        public static final int CLAW_RIGHT_WHEELS = 2;
  }

  public static final class Swerve{

    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KA = 0;
    public static final double DRIVE_KV = 0;
    public static final double MAX_SPEED = 0;
    public static final double DRIVE_GEAR_RATIO = (6.12 / 1); //6.12 : 1
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4 * Math.PI);
    public static final double ANGLE_GEAR_RATIO = ((150 / 7) / 1); //150/7 : 1
    public static final boolean CANCONDER_INVERT = false;
    
    public static final boolean DRIVE_MOTOR_INVERT = false;
    public static final boolean ANGLE_MOTOR_INVERT = false;
    public static final double DRIVE_CLOSED_LOOP_RAMP = 0;
    public static final double DRIVE_OPEN_LOOP_RAMP = 0;
    public static final int ANGLE_MOTOR_SMART_CURRENT = 25;
    public static final double ANGLE_MOTOR_SECONDARY_LIMIT = 40;
    public static final int DRIVE_MOTOR_SMART_CURRENT = 35;
    public static final double DRIVE_MOTOR_SECONDARY_LIMIT = 60;
    public static final double ANGLE_MOTOR_KP = 0;
    public static final double ANGLE_MOTOR_KI = 0;
    public static final double ANGLE_MOTOR_KD = 0;
    public static final double DRIVE_MOTOR_KP = 0;
    public static final double DRIVE_MOTOR_KI = 0;
    public static final double DRIVE_MOTOR_KD = 0;
    public static final double DRIVE_MOTOR_KF = 0;
    public static final double DRIVE_MOTOR_MIN_OUTPUT = -1;
    public static final double DRIVE_MOTOR_MAX_OUTPUT = 1;
    public static final double DRIVE_MOTOR_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VELOCITY_CONVERSION = (WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO) / 60;
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = null;
public static final boolean INVERT_GYRO = false;
public static final double MAX_ANGULAR_VELOCITY = 0;
public static final double KP_APRIL_TAGS = 0;

        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
  }

  public static final class AutoConstants {
    public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 0;
    public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 0;
  }
}