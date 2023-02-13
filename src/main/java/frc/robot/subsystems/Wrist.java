// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public ArmFeedforward feedForward = new ArmFeedforward(
            Constants.Wrist.WRIST_KS, 
            Constants.Wrist.WRIST_KG, 
            Constants.Wrist.WRIST_KV);
     
  private ProfiledPIDController controller = new ProfiledPIDController(
            Constants.Wrist.WRIST_KP, 
            Constants.Wrist.WRIST_KI, 
            Constants.Wrist.WRIST_KD, 
            new TrapezoidProfile.Constraints(
                Constants.Wrist.WRIST_MAX_VEL, 
                Constants.Wrist.WRIST_MAX_ACCEL));

  public static double setpointWrist = 0;

  public Wrist() {
    wrist.setNeutralMode(NeutralMode.Brake);

    // wrist.config_kP(0, Constants.Wrist.KP);
    // wrist.config_kP(0, Constants.Wrist.KI);
    // wrist.config_kP(0, Constants.Wrist.KD);

    wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    wrist.configForwardSoftLimitEnable(false);
    wrist.configReverseSoftLimitEnable(false);
    wrist.configForwardSoftLimitThreshold(0);
    wrist.configReverseSoftLimitThreshold(0);

    wrist.setInverted(false);
  }

  public WristState getState(){
    return wristState;
  }

  public void setState(WristState state){
    wristState = state;
  }

  public double degreesToTicks(double degrees) {
    double armRotations = degrees / 360;
    double motorRotations = armRotations * Constants.Wrist.WRIST_GEAR_RATIO;
    double motorTicks = motorRotations * 4096;
    return motorTicks;
  }

  // give the encoder value to get degrees
  public double ticksToDegrees(double ticks) {
    double motorRotations = ticks / 4096;
    double armRotations = motorRotations / Constants.Wrist.WRIST_GEAR_RATIO;
    double armDegrees = armRotations * 360;
    return armDegrees; 
  }
  
  public void setWristPosition() {
    controller.setGoal(setpointWrist);
    // wrist.set(ControlMode.Position, degreesToTicks(degrees));
    wrist.set(ControlMode.PercentOutput, controller.calculate(wrist.getSelectedSensorPosition()) + 
                                   feedForward.calculate(Math.toRadians(controller.getSetpoint().position), 
                                                              controller.getSetpoint().velocity));
  }
  public static double getSetpoint(){
    return setpointWrist;
  }
  
  public static void setSetpoint(double setpoint){
    setpointWrist = setpoint;
  }
  
  @Override
  public void periodic() {
    switch(wristState) {
      case IDLE:
        setWristPosition();
        break;
      case MANUAL:
        System.out.println("lol no");
        break;
      case CLOSED_LOOP:
        setWristPosition();
        break;
    }
  }
}
