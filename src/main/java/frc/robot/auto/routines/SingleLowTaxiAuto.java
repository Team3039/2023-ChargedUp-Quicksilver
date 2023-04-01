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
import frc.robot.auto.commands.SetWristIdleMode;
import frc.robot.commands.ActuateWristToSetpoint;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleLowTaxiAuto extends SequentialCommandGroup {
  /** Creates a new ChargeStationAuto. */
  public SingleLowTaxiAuto(Drive swerve) {

    SwerveAutoBuilder autoBuilder = PPTrajectoryGenerator.getAutoBuilder();

    Command driveOut = autoBuilder.fullAuto(PPTrajectoryGenerator.getDriveOut());

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getDriveOut().getInitialHolonomicPose())),
        new ParallelDeadlineGroup(
					new WaitCommand(.3), 
					new SetClawIntakeMode()),
				new InstantCommand(() -> RobotContainer.claw.setState(ClawState.PASSIVE)),
        new InstantCommand(() -> swerve.setGyro(0)),
        new ActuateWristToSetpoint(20, 3),
        new SetClawReleaseMode(),
        new WaitCommand(0.5),
        new SetClawIdleMode(),
        new SetWristIdleMode(),
        driveOut,
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
        new RotateRobotToSetpoint(swerve, 180, 5.0),
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)));
  }
}
