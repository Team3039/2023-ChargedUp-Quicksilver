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
import frc.robot.RobotContainer;
import frc.robot.auto.PPTrajectoryGenerator;
import frc.robot.auto.commands.RotateRobotToSetpoint;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawIntakeMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridConeAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridCubeAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateToIdleAuto;
import frc.robot.commands.ActuateWristToSetpoint;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Wrist.WristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomThreePieceDispenseAuto extends SequentialCommandGroup {

  /** Creates a new TopTwoPieceAuto. */
  public BottomThreePieceDispenseAuto(Drive swerve) {

    SwerveAutoBuilder autoBuilder = PPTrajectoryGenerator.getAutoBuilder();

    Command driveOverWireCover = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverDriveOverWireCover());
    Command grabFirstPiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverGrabFirstPiece());
    Command driveToDispensePiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverDriveToDispensePiece());
    Command grabSecondPiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverGrabSecondPiece());
    Command driveBackToWireCover = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverDriveBackToWireCover());
    Command driveBackOverWireCover = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverDriveBackOverWireCover());
 

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getWireCoverDriveOverWireCover().getInitialHolonomicPose())),
        new ParallelDeadlineGroup(
			new WaitCommand(.15), 
			new InstantCommand(() -> RobotContainer.claw.setState(ClawState.INTAKE))),
        new SetClawIdleMode(), 
        new ActuateLowToHighGridConeAuto(),     
        new SetClawReleaseMode(),
        new WaitCommand(0.15),
        new ActuateToIdleAuto(),
        new WaitCommand(.4),
        driveOverWireCover,
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        // new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getWireCoverGrabFirstPiece().getInitialHolonomicPose())),
        new ParallelDeadlineGroup(
            grabFirstPiece,
            new SetClawIntakeMode()),
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new InstantCommand(() -> RobotContainer.claw.setState(ClawState.PASSIVE)),
        // new RotateRobotToSetpoint(swerve, 180.0, 0.7),
        // new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        // new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getWireCoverDriveToDispensePiece().getInitialHolonomicPose())),
        driveToDispensePiece,
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        // new RotateRobotToSetpoint(swerve, 0, 1.0),
        // new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new ActuateWristToSetpoint(60, 20),    
        new SetClawReleaseMode(),    
        new WaitCommand(0.3),
        new SetClawIdleMode(),
        new ActuateToIdleAuto(),
        // new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getWireCoverGrabSecondPiece().getInitialHolonomicPose())),
        new ParallelDeadlineGroup(
            grabSecondPiece,
            new SequentialCommandGroup(
                new WaitCommand(0.05),
                new SetClawIntakeMode())),
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new InstantCommand(() -> RobotContainer.wrist.setState(WristState.PASSIVE)),
        new InstantCommand(() -> RobotContainer.claw.setState(ClawState.PASSIVE)),
        driveBackToWireCover,
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        // new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getWireCoverDriveBackOverWireCover().getInitialHolonomicPose())),
        new ParallelDeadlineGroup(
          driveBackOverWireCover,
          new SequentialCommandGroup(
            new WaitCommand(0.5),
            new ActuateLowToHighGridCubeAuto())),
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new RotateRobotToSetpoint(swerve,  0, 0.7),
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new SetClawReleaseMode(),
        new WaitCommand(0.1),
        new SetClawIdleMode(),
        new ActuateToIdleAuto()
    
        );
  }
}
