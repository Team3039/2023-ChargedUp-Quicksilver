// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.auto.commands.SetWristIdleMode;
import frc.robot.commands.ActuateWristToSetpoint;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleLowAuto extends SequentialCommandGroup {
  /** Creates a new ChargeStationAuto. */
  public SingleLowAuto(Drive s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
      new InstantCommand(() -> s_Swerve.setGyro(0)),
      new ActuateWristToSetpoint(20),
      new SetClawReleaseMode(),
      new WaitCommand(0.5),
      new SetClawIdleMode(),
      new SetWristIdleMode(),
      new InstantCommand(() -> s_Swerve.setGyro(180))
      // new DriveOntoChargeStation(s_Swerve),
      // new ChargeStationBalance(s_Swerve),
      // new LockWheels(s_Swerve)
    );
  }
}
