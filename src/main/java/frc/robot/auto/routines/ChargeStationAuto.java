// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.ChargeStationBalance;
import frc.robot.auto.commands.DriveOntoChargeStation;
import frc.robot.auto.commands.LockWheels;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.commands.ElevatorRoutines.ActuateLowToHighGrid;
import frc.robot.commands.ElevatorRoutines.ActuateToIdle;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeStationAuto extends SequentialCommandGroup {
  /** Creates a new ChargeStationAuto. */
  public ChargeStationAuto(Drive s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
      new ActuateLowToHighGrid(),
      new SetClawReleaseMode(),
      new WaitCommand(0.7),
      new SetClawIdleMode(),
      new ActuateToIdle(),
      new DriveOntoChargeStation(s_Swerve),
      new ChargeStationBalance(s_Swerve),
      new LockWheels(s_Swerve)
    );
  }
}
