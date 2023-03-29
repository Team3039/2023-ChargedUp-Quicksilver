// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class SetClawReleaseMode extends CommandBase {

  private Timer timer = new Timer();

  public SetClawReleaseMode() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
      RobotContainer.claw.setState(ClawState.RELEASE);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
