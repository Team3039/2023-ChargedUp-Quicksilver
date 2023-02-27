// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawIntakeMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.commands.ElevatorRoutines.ActuateLowToHighGrid;
import frc.robot.commands.ElevatorRoutines.ActuateToIdle;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomTwoPieceYP extends SequentialCommandGroup {
    
    public BottomTwoPieceYP(Drive swerve) {
        addRequirements(swerve);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
                Constants.AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand startToGamePiece = new SwerveControllerCommand(
                frc.robot.auto.TrajectoryGenerator.getBottomStartToBottomPiece(),
                swerve::getPose,
                Constants.Swerve.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
                thetaController,
                Drive.getSwerveHeadingSupplier(180),
                swerve::setModuleStates,
                swerve);

        SwerveControllerCommand gamePieceToBottomShelf = new SwerveControllerCommand(
                frc.robot.auto.TrajectoryGenerator.getBottomPieceToBottomShelf(),
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
                new InstantCommand(() -> swerve.resetOdometry(frc.robot.auto.TrajectoryGenerator.getGamePieceToStart().getInitialPose())),
                new ActuateLowToHighGrid(),
                new SetClawReleaseMode(),
                new WaitCommand(1),
                new SetClawIdleMode(),
                new ActuateToIdle(),
                new SetClawIntakeMode(),
                startToGamePiece,
                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, true)),
                new WaitCommand(1),
                gamePieceToBottomShelf,
                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, true)),
                new ActuateLowToHighGrid(),
                new SetClawReleaseMode(),
                new WaitCommand(1),
                new SetClawIdleMode(),
                new ActuateToIdle()
                );

    }
}