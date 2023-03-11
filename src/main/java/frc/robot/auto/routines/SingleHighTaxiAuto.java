// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.commands.ElevatorRoutines.ActuateLowToHighGrid;
import frc.robot.commands.ElevatorRoutines.ActuateToIdle;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleHighTaxiAuto extends SequentialCommandGroup {
  /** Creates a new ChargeStationAuto. */
  public SingleHighTaxiAuto(Drive swerve) {

    var thetaController = new ProfiledPIDController(
      Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
      Constants.AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand driveStraight = new SwerveControllerCommand(
    //   frc.robot.auto.TrajectoryGenerator.getstartToGamePiece(),
    //   swerve::getPose,
    //   Constants.Swerve.SWERVE_KINEMATICS,
    //   new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
    //   new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
    //   thetaController,
    //   Drive.getSwerveHeadingSupplier(0),
    //   swerve::setModuleStates,
    //   swerve);

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(new Pose2d())),
      new InstantCommand(() -> swerve.setGyro(0)),
      new ActuateLowToHighGrid(),
      new SetClawReleaseMode(),
      new WaitCommand(0.5),
      new SetClawIdleMode(),
      new ActuateToIdle(),
      // new ParallelRaceGroup(driveStraight, new WaitCommand(3.3)),
      new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
      new InstantCommand(() -> swerve.setGyro(180))
    );
  }
}
