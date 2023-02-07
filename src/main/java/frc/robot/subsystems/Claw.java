// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.ConnectException;

import javax.swing.text.PasswordView;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  public enum ClawState {
    IDLE,
    PASSIVE,
    CONE,
    CUBE
  }

  public ClawState clawState = ClawState.IDLE;
  
  public CANSparkMax leftWheels = new CANSparkMax(Constants.Ports.CLAW_LEFT_WHEELS, MotorType.kBrushless);
  public CANSparkMax rightWheels = new CANSparkMax(Constants.Ports.CLAW_RIGHT_WHEELS, MotorType.kBrushless);
  public Solenoid snapper = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.CLAW_SOLENOID);
  public DigitalInput beamTrigger = new DigitalInput(Constants.Ports.CLAW_BEAM_BREAK);

  public Claw() {
    leftWheels.setIdleMode(IdleMode.kBrake);
    rightWheels.setIdleMode(IdleMode.kBrake);
  }

  public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
    leftWheels.set(leftSpeed);
    rightWheels.set(rightSpeed);
  }

  @Override
  public void periodic() {
    switch (clawState) {
      case IDLE:
        setWheelSpeeds(0, 0);
        break;
      case PASSIVE:
        setWheelSpeeds(-0.05, 0.05);
        break;
      case CONE:
        break;
      case CUBE:
        break;
    }
  }
}
