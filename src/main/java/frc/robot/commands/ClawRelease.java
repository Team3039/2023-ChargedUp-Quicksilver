// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class ClawRelease extends CommandBase {
  Timer timer = new Timer();
  /** Creates a new ClawRelease. */
  public ClawRelease() {
   addRequirements(RobotContainer.claw, RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.elevator.getPosition() < 10) {
      timer.reset();
      timer.start();
      Wrist.setSetpoint(20);
      RobotContainer.wrist.setState(WristState.POSITION);
    }
    else {
      RobotContainer.claw.setState(ClawState.RELEASE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(0.3)) {
      RobotContainer.claw.setState(ClawState.RELEASE);
      timer.stop();
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.claw.setState(ClawState.IDLE);
    RobotContainer.wrist.setState(WristState.IDLE);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
