// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  public enum WristState {
    IDLE,
    MANUAL,
    CLOSED_LOOP
  }

  public WristState wristState;

  public TalonSRX wrist = new TalonSRX(Constants.Ports.WRIST);

  public ArmFeedforward wristFeedForward = new ArmFeedforward(Constants.Wrist.KS, Constants.Wrist.KG, Constants.Wrist.KV);

  public double setpointWrist = 0;

  public Wrist() {
    wrist.setNeutralMode(NeutralMode.Brake);

    wrist.config_kP(0, Constants.Wrist.KP);
    wrist.config_kP(0, Constants.Wrist.KI);
    wrist.config_kP(0, Constants.Wrist.KD);

    wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    wrist.configForwardSoftLimitEnable(false);
    wrist.configReverseSoftLimitEnable(false);
    wrist.configForwardSoftLimitThreshold(0);
    wrist.configReverseSoftLimitThreshold(0);

    wrist.setInverted(false);
  }

  public double degreesToTicks(double degrees) {
    return degrees * (4096 / 360.0);
  }

  public double getCurrentAngle() {
    return (wrist.getSelectedSensorPosition() / 4096) * 360;
  }
  
  public void setWristPosition(double degrees) {
    wrist.set(ControlMode.Position, degreesToTicks(degrees));
  }
  
  @Override
  public void periodic() {
    switch(wristState) {
      case IDLE:
        setWristPosition(0);
        break;
      case MANUAL:
        System.out.println("lol no");
        break;
      case CLOSED_LOOP:
        setWristPosition(setpointWrist);
        break;
    }
  }
}
