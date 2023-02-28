// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class RotateRobotToSetpoint extends CommandBase {
  
  Drive drive;
  double setpoint;
  double rotation;

  private PIDController rotController = new PIDController(0.04, 0.6, 0.00);

  public RotateRobotToSetpoint(Drive drive, double setpoint) {
    addRequirements(drive);
    this.drive = drive;
    this.setpoint = setpoint;

    rotController.reset();
    rotController.setIntegratorRange(-0.2, 0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotation = rotController.calculate(Math.abs(drive.getAngle()), setpoint);
        if (drive.getAngle() < 0) {
            rotation *= -1;
        }
        rotation = MathUtil.clamp(rotation, -3.2, 3.2);

        drive.drive(new Translation2d(), rotation, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(setpoint - drive.getYaw().getDegrees()) < 2.0;
  }
}
