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
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToPreScoreAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateToIdleAuto;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomTwoPieceWithGrabAuto extends SequentialCommandGroup {

	public BottomTwoPieceWithGrabAuto(Drive swerve) {

		SwerveAutoBuilder autoBuilder = PPTrajectoryGenerator.getAutoBuilder();

		Command bottomTwoPiece = autoBuilder.fullAuto(PPTrajectoryGenerator.getBottomPathTwoPiece());
		Command bottomPathDriveOut = autoBuilder.fullAuto(PPTrajectoryGenerator.getBottomPathDriveOut());

		addCommands(
				new InstantCommand(
						() -> swerve.resetOdometry(PPTrajectoryGenerator.getBottomPathTwoPiece().getInitialHolonomicPose())),
				new ActuateLowToHighGridAuto(),
				new SetClawReleaseMode(),
				new WaitCommand(0.5),
				new ActuateToIdleAuto(),
				new ParallelDeadlineGroup(
						bottomTwoPiece,
						new SequentialCommandGroup(
								new WaitCommand(.8),
								new SetClawIntakeMode()),
						new SequentialCommandGroup(
								new WaitCommand(4),
								new ActuateLowToPreScoreAuto())),
				new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
				new RotateRobotToSetpoint(swerve, 0, 0.5),
				new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
				new InstantCommand(() -> RobotContainer.claw.setState(ClawState.PASSIVE)),
				new ActuateLowToHighGridAuto(),
				new SetClawReleaseMode(),
				new WaitCommand(0.5),
				new SetClawIdleMode(),
				new ActuateToIdleAuto(),
				new ParallelDeadlineGroup(
						bottomPathDriveOut,
						new SequentialCommandGroup(
								new WaitCommand(.8),
								new SetClawIntakeMode())),
				new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)));

	}
}