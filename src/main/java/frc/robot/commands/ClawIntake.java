// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class ClawIntake extends CommandBase {
  /** Creates a new ClawIntake. */
  double setpointE;
  double setpointW;
  boolean isSnappingAllowed;
  public ClawIntake(double setpointE, double setpointW, boolean isSnappingAllowed) {
    addRequirements(RobotContainer.claw, RobotContainer.wrist);
    this.setpointE = setpointE;
    this.setpointW = setpointW;
    this.isSnappingAllowed = isSnappingAllowed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.claw.setSnapper(true);
    RobotContainer.claw.setState(ClawState.INTAKE);
    if (RobotContainer.elevator.getPosition() < 30) {
    Wrist.setSetpoint(setpointW);
    RobotContainer.wrist.setState(WristState.POSITION);
    new WaitCommand(0.2);
    Elevator.setSetpoint(setpointE);
    RobotContainer.elevator.setState(ElevatorState.POSITION);
    RobotContainer.claw.isSnappingAllowed(isSnappingAllowed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotContainer.claw.isIntakeDeactivated()) {
      RobotContainer.claw.setState(ClawState.IDLE);
      RobotContainer.wrist.setState(WristState.IDLE);
    } else {
      RobotContainer.claw.setState(ClawState.PASSIVE);
      RobotContainer.wrist.setState(WristState.PASSIVE);
    }
    RobotContainer.claw.isSnappingAllowed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
