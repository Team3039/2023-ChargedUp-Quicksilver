// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends SequentialCommandGroup {
    
    /** Creates a new DriveStraight. */
    public DriveStraight(Drive swerve) {
        addRequirements(swerve);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
                Constants.AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand driveStraight = new SwerveControllerCommand(
                frc.robot.auto.TrajectoryGenerator.getstartToGamePiece(),
                swerve::getPose,
                Constants.Swerve.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
                thetaController,
                Drive.getSwerveHeadingSupplier(0),
                swerve::setModuleStates,
                swerve);

        SwerveControllerCommand driveReverse = new SwerveControllerCommand(
                frc.robot.auto.TrajectoryGenerator.getGamePieceToStart(),
                swerve::getPose,
                Constants.Swerve.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
                thetaController,
                Drive.getSwerveHeadingSupplier(0),
                swerve::setModuleStates,
                swerve);


        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new InstantCommand(() -> swerve.resetOdometry(frc.robot.auto.TrajectoryGenerator.getstartToGamePiece().getInitialPose())),
                driveStraight,
                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, true)),
                new WaitCommand(0.2),
                driveReverse,
                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, true))
                );

    }
}