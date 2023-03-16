// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.PPTrajectoryGenerator;
import frc.robot.auto.commands.LockWheels;
import frc.robot.auto.commands.RotateRobotToSetpoint;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawIntakeMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateToIdleAuto;
import frc.robot.auto.commands.chargestation.Lpath.ChargeStationBalanceLPath;
import frc.robot.auto.commands.chargestation.Lpath.DriveOntoChargeStationLPath;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeStationTopLPath extends SequentialCommandGroup {

  public ChargeStationTopLPath(Drive swerve) {
  SwerveAutoBuilder autoBuilder = PPTrajectoryGenerator.getAutoBuilder();

  Command topLPath = autoBuilder.fullAuto(PPTrajectoryGenerator.getChargeStationTopLPath()); 

    addCommands(
    new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getChargeStationTopLPath().getInitialHolonomicPose())),
    new InstantCommand(() -> swerve.setGyro(0)),
    new ActuateLowToHighGridAuto(),
    new SetClawReleaseMode(),
    new WaitCommand(0.5),
    new SetClawIdleMode(),
    new ActuateToIdleAuto(),
    new ParallelDeadlineGroup(
      topLPath,
      new SequentialCommandGroup(
          new WaitCommand(.8),
          new SetClawIntakeMode())),
    new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
    new RotateRobotToSetpoint(swerve, 180, 1),
    new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
    new DriveOntoChargeStationLPath(swerve),
    new ChargeStationBalanceLPath(swerve),
    new LockWheels(swerve));
  }
}
