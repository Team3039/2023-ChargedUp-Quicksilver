// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetLEDS extends CommandBase {
  /** Creates a new DesiresCone. */
  public SetLEDS() {
    addRequirements(RobotContainer.leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.leds.getDesiresCone() == false && RobotContainer.leds.getDesiresCube() == false) {
      RobotContainer.leds.setDesiredPiece(true, false);

    }
    else if (RobotContainer.leds.getDesiresCone() == true) {
      RobotContainer.leds.setDesiredPiece(false, true);

    }
    else {
      RobotContainer.leds.setDesiredPiece(false, false);

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
