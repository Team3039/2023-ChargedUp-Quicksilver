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
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawIntakeMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridConeAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToPreScoreAuto;
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

    Command driveOverWireCover = autoBuilder.fullAuto(PPTrajectoryGenerator.getDriveOverWireCover());
    Command grabFirstPiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverGrabFirstPiece());
    Command driveToDispensePiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverDriveToDispensePiece());
    Command grabSecondPiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverGrabSecondPiece());
    Command driveBackToWireCover = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverDriveBackToWireCover());
    Command driveBackOverWireCover = autoBuilder.fullAuto(PPTrajectoryGenerator.getWireCoverDriveBackOverWireCover());
 

    addCommands(
        // new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getDriveOverWireCover().getInitialHolonomicPose())),
        // new ParallelDeadlineGroup(
		// 	new WaitCommand(.3), 
		// 	new SetClawIntakeMode()),
        // new SetClawIdleMode(), 
        // new ActuateLowToHighGridConeAuto(),     
        // new SetClawReleaseMode(),
        // new WaitCommand(0.15),
        // new ActuateToIdleAuto(),
        // driveOverWireCover,
        // new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        // new SetClawIntakeMode(),
        // new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getDriveOverWireCover().getInitialHolonomicPose())),
        // grabFirstPiece,
        // new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        // new InstantCommand(() -> RobotContainer.claw.setState(ClawState.PASSIVE)),
        // driveToDispensePiece,
        // new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        // new ActuateWristToSetpoint(60, 20),    
        // new SetClawReleaseMode(),    
        // new WaitCommand(0.1),
        // new SetClawIdleMode(),
        // new ActuateToIdleAuto(),
        // new ParallelDeadlineGroup(
        //     TopThirdPiece,
        //     new SequentialCommandGroup(
        //         new WaitCommand(1),
        //         new SetClawIntakeMode()),
        //     new SequentialCommandGroup(
        //         new WaitCommand(3),
        //         new InstantCommand(() -> RobotContainer.wrist.setState(WristState.PASSIVE))),
        //     new SequentialCommandGroup(
        //         new WaitCommand(3.7),
        //         new ActuateLowToPreScoreAuto())),
        // new SetClawReleaseMode(),
        // new WaitCommand(0.1),
        // new SetClawIdleMode(),
        // new ActuateToIdleAuto(),        
        // new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false))
        );
  }
}
