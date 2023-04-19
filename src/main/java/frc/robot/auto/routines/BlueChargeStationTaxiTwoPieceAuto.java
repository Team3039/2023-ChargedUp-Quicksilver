// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.auto.PPTrajectoryGenerator;
import frc.robot.auto.commands.LockWheels;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawIntakeMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridConeAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridCubeAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateToIdleAuto;
import frc.robot.auto.commands.chargestation.normal.DriveOntoChargeStation;
import frc.robot.auto.commands.chargestation.taxi.DrivePastChargeStationTaxiForward;
import frc.robot.auto.commands.chargestation.taxi.DrivePastChargeStationTaxiReverse;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Wrist.WristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueChargeStationTaxiTwoPieceAuto extends SequentialCommandGroup {

  SwerveAutoBuilder autoBuilder = PPTrajectoryGenerator.getAutoBuilder();

  Command grabPiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getMidTaxiGrabPiece());
  /** Creates a new ChargeStationAuto. */
  public BlueChargeStationTaxiTwoPieceAuto(Drive swerve) {
    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d())),
        new InstantCommand(() -> swerve.setGyro(0)),
        new ParallelDeadlineGroup(
					new WaitCommand(.15), 
          new InstantCommand(() -> RobotContainer.claw.setState(ClawState.INTAKE))),
				new InstantCommand(() -> RobotContainer.claw.setState(ClawState.IDLE)),
        new ActuateLowToHighGridConeAuto(),
        new SetClawReleaseMode(),
        new WaitCommand(0.1),
        new SetClawIdleMode(),
        new ActuateToIdleAuto(),
        new DrivePastChargeStationTaxiReverse(swerve),
        new SetClawIntakeMode(),
        grabPiece,
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new InstantCommand(() -> RobotContainer.wrist.setState(WristState.PASSIVE)),
        new DrivePastChargeStationTaxiForward(swerve),
        new ParallelDeadlineGroup(
          new DriveToPose(swerve, new Pose2d(1.9, 2.7, new Rotation2d(0)), false),
          new SequentialCommandGroup(
            new WaitCommand(0.3),
            new ActuateLowToHighGridCubeAuto())),
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new SetClawReleaseMode(),
        new WaitCommand(0.1),
        new SetClawIdleMode(),
        new ActuateToIdleAuto(),
        // new WaitCommand(1),
        new DriveOntoChargeStation(swerve),
        // new ChargeStationBalanceTaxi(swerve),
        new LockWheels(swerve),
        new InstantCommand(() -> swerve.setGyro(180)));
  }
}
