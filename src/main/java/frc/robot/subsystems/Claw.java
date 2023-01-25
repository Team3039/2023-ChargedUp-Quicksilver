// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public CANSparkMax leftWheels = new CANSparkMax();
public CANSparkMax rightWheels = new CANSparkMax();

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  public Claw() {}

  public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
    leftWheels.set(leftSpeed);
    rightWheels.set(rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setWheelSpeeds(.1, -.1)
  }
}
