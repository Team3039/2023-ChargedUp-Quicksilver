// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {

  public enum WristState {
    IDLE,
    MANUAL,
    POSITION
  }

  public WristState wristState = WristState.IDLE;

  public TalonSRX wrist = new TalonSRX(Constants.Ports.WRIST);

  public ArmFeedforward feedForward = new ArmFeedforward(
      Constants.Wrist.WRIST_KS,
      Constants.Wrist.WRIST_KG,
      Constants.Wrist.WRIST_KV);

  public ProfiledPIDController profiledController = new ProfiledPIDController(
      Constants.Wrist.WRIST_KP,
      Constants.Wrist.WRIST_KI,
      Constants.Wrist.WRIST_KD,
      new TrapezoidProfile.Constraints(
          Constants.Wrist.WRIST_MAX_VEL,
          Constants.Wrist.WRIST_MAX_ACCEL));

  public PIDController controller = new PIDController(
      Constants.Wrist.WRIST_KP,
      Constants.Wrist.WRIST_KI,
      Constants.Wrist.WRIST_KD);

  public static double setpointWrist = 0;

  public Wrist() {
    wrist.setNeutralMode(NeutralMode.Brake);

    // wrist.config_kP(0, Constants.Wrist.KP);
    // wrist.config_kP(0, Constants.Wrist.KI);
    // wrist.config_kP(0, Constants.Wrist.KD);

    // Wrist must start in the vertical position in order to be legal. DONT FORGET
    // TO DO THIS PLS
    wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    wrist.setSelectedSensorPosition(degreesToTicks(20));

    wrist.configForwardSoftLimitEnable(true);
    wrist.configReverseSoftLimitEnable(true);
    wrist.configForwardSoftLimitThreshold(degreesToTicks(90));
    wrist.configReverseSoftLimitThreshold(degreesToTicks(-20));

    wrist.setInverted(true);

    controller.setTolerance(3);
    profiledController.setTolerance(3);
  }

  public WristState getState() {
    return wristState;
  }

  public void setState(WristState state) {
    wristState = state;
  }

  public double degreesToTicks(double degrees) {
    double armRotations = degrees / 360;
    double ticks = armRotations * 4096;
    return ticks;
  }

  // give the encoder value to get degrees
  public double ticksToDegrees(double ticks) {
    double armRotations = ticks / 4096;
    double armDegrees = armRotations * 360;
    return armDegrees;
  }

  public void setWristPosition(boolean isProfiled) {
    if (isProfiled) {
      profiledController.setGoal(setpointWrist);
      // wrist.set(ControlMode.Position, degreesToTicks(degrees));
      wrist.set(ControlMode.PercentOutput, profiledController.calculate(wrist.getSelectedSensorPosition()) +
          feedForward.calculate(Math.toRadians(profiledController.getSetpoint().position),
              profiledController.getSetpoint().velocity));
    } else {
      wrist.set(ControlMode.PercentOutput, MathUtil.clamp(controller.calculate(
          ticksToDegrees(wrist.getSelectedSensorPosition()),
          setpointWrist), -.2, .2),
          DemandType.ArbitraryFeedForward,
          Math.cos(Math.toRadians(ticksToDegrees(wrist.getSelectedSensorPosition()))) * Constants.Wrist.WRIST_KG +
              Constants.Wrist.WRIST_KS);
    }
  }

  public void setWristPercent(double percent) {
    wrist.set(ControlMode.PercentOutput, percent +
        Math.cos(Math.toRadians(ticksToDegrees(wrist.getSelectedSensorPosition()))) * Constants.Wrist.WRIST_KG +
        Constants.Wrist.WRIST_KS);
  }

  public static double getSetpoint() {
    return setpointWrist;
  }

  public static void setSetpoint(double setpoint) {
    setpointWrist = setpoint;
  }

  public boolean isAtSetpoint(boolean isProfiled) {
    return isProfiled ? profiledController.atSetpoint() : controller.atSetpoint();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Wrist Absolute Encoder",
    // wrist.getSelectedSensorPosition());
    // System.out.println(ticksToDegrees(wrist.getSelectedSensorPosition()));
    // System.out.println(wrist.getMotorOutputPercent());
    // SmartDashboard.putNumber("Wrist Currnent Input", wrist.getSupplyCurrent());
    // SmartDashboard.putNumber("Wrist Current Output", wrist.getStatorCurrent());
    // System.out.println(setpointWrist);

    switch (wristState) {
      case IDLE:
        // System.out.println(setpointWrist);
        if (RobotContainer.claw.isIntakeDeactivated()) {
          setSetpoint(20);
        } else {
          setSetpoint(0);
        }
        setWristPosition(false);
        break;
      case MANUAL:
        break;
      case POSITION:
        setWristPosition(false);
        break;
    }
  }
}
