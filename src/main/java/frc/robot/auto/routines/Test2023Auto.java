// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.auto.PPTrajectoryGenerator;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test2023Auto extends SequentialCommandGroup {
    
    public Test2023Auto(Drive swerve) {

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
                Constants.AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand testCommandOne = new SwerveControllerCommand(
        //         frc.robot.auto.TrajectoryGenerator.getstartToGamePiece(),
        //         swerve::getPose,
        //         Constants.Swerve.SWERVE_KINEMATICS,
        //         new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
        //         new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
        //         thetaController,
        //         Drive.getSwerveHeadingSupplier(90),
        //         swerve::setModuleStates,
        //         swerve);

        // SwerveControllerCommand testCommandTwo = new SwerveControllerCommand(
        //         frc.robot.auto.TrajectoryGenerator.getGamePieceToStart(),
        //         swerve::getPose,
        //         Constants.Swerve.SWERVE_KINEMATICS,
        //         new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
        //         new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
        //         thetaController,
        //         Drive.getSwerveHeadingSupplier(180),
        //         swerve::setModuleStates,
        //         swerve);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        swerve::getPose,
        swerve::resetOdometry,
        Constants.Swerve.SWERVE_KINEMATICS,
        new PIDConstants(1.0, 0.0, 0.0),
        new PIDConstants(0.4, 0.0, 0.0),
        swerve::setModuleStates,
        eventMap,
        true,
        swerve
        );

        PPSwerveControllerCommand testCommandOne = new PPSwerveControllerCommand(
            PPTrajectoryGenerator.getForwardTestPath(),
            swerve::getPose,
            Constants.Swerve.SWERVE_KINEMATICS,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            swerve::setModuleStates,
            true,
            swerve);

        PPSwerveControllerCommand testCommandTwo = new PPSwerveControllerCommand(
            PPTrajectoryGenerator.getReverseTestPath(),
            swerve::getPose,
            Constants.Swerve.SWERVE_KINEMATICS,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            swerve::setModuleStates,
            true,
            swerve);

        SwerveControllerCommand testCommandThree = new SwerveControllerCommand(
            frc.robot.auto.TrajectoryGenerator.getstartToGamePiece(),
            swerve::getPose,
            Constants.Swerve.SWERVE_KINEMATICS,
            new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
            thetaController,
            Drive.getSwerveHeadingSupplier(0),
            swerve::setModuleStates,
            swerve);

        
        Command forwardTestCommandOne = autoBuilder.fullAuto(PPTrajectoryGenerator.getForwardTestPath());
        Command reverseTestCommandOne = autoBuilder.fullAuto(PPTrajectoryGenerator.getReverseTestPath());

        Command bottomThreePieceTest = autoBuilder.fullAuto(PPTrajectoryGenerator.getBottomPathThreePiece());
        Command topThreePieceTest = autoBuilder.fullAuto(PPTrajectoryGenerator.getTopPathThreePiece());
        

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new InstantCommand(() -> swerve.resetOdometry(PPTrajectoryGenerator.getBottomPathThreePiece().getInitialHolonomicPose())),  
                // new InstantCommand(() -> swerve.resetOdometry(new Pose2d())),
                // forwardTestCommandOne,
                // reverseTestCommandOne,
                bottomThreePieceTest,
                // topThreePieceTest,
                // testCommandOne,
                // testCommandTwo,
                // testCommandThree,
                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false)),
                new PrintCommand("hello this is the auto speaking, LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                
              );

    }
}