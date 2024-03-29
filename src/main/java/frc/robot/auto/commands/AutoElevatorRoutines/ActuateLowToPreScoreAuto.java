// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands.AutoElevatorRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateElevatorToSetpoint;
import frc.robot.commands.ActuateWristToSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActuateLowToPreScoreAuto extends SequentialCommandGroup {
  /** Creates a new ActuateLowToSingleStation. */
  public ActuateLowToPreScoreAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActuateWristToSetpoint(65, 5),
      new ActuateElevatorToSetpoint(35, 3)
    );
  }
}
