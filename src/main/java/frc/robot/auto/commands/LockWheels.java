// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class LockWheels extends CommandBase {

 Drive drive;
  public LockWheels(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(new Translation2d(), 0, true, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.lockModules();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(Math.abs(drive.getPositions()[0].angle.getDegrees() - 90)) < 10 ||
        Math.abs(Math.abs(drive.getPositions()[0].angle.getDegrees() - 270)) < 10) {
      return true;
    }
    return false;

  }
}
