// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawIntakeMode;
import frc.robot.auto.commands.SetElevatorIdleMode;
import frc.robot.auto.commands.SetElevatorPositionMode;
import frc.robot.auto.commands.SetWristIdleMode;
import frc.robot.auto.commands.SetWristPositionMode;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class PPTrajectoryGenerator {
    public static Drive swerve = RobotContainer.drive;
    public static PPTrajectoryGenerator INSTANCE = new PPTrajectoryGenerator();

    public static HashMap<String, Command> eventMap;

    public static SwerveAutoBuilder autoBuilder;

    public static PathPlannerTrajectory getTestPath() {
        return PathPlanner.loadPath("Test Code", 
                new PathConstraints(1.0, 3));
    }
    
    public static PathPlannerTrajectory getForwardTestPath() {
        return PathPlanner.loadPath("Forward Path", 
                new PathConstraints(3.0, 3));
    }
    
    public static PathPlannerTrajectory getReverseTestPath() {
        return PathPlanner.loadPath("Reverse Path", 
                new PathConstraints(3.0, 3));
    }

    public static PathPlannerTrajectory getBottomPathThreePiece() {
        return PathPlanner.loadPath("Bottom 3 Piece YYP", 
            new PathConstraints(3.0, 5));
    }

    public static PathPlannerTrajectory getTopPathThreePiece() {
        return PathPlanner.loadPath("Top 3 Piece YYP", 
            new PathConstraints(3.0, 5));
    }


    public PPTrajectoryGenerator() {
        eventMap = new HashMap<>();
        eventMap.put("Claw Idle", new SetClawIdleMode());
        eventMap.put("Claw Intake", new SetClawIntakeMode());
        eventMap.put("Elevator Idle", new SetElevatorIdleMode());
        eventMap.put("Elevator Mid Grid", new SetElevatorPositionMode(Constants.Elevator.MID_GRID_SETPOINT));
        eventMap.put("Elevator High Grid", new SetElevatorPositionMode(Constants.Elevator.HIGH_GRID_SETPOINT));
        eventMap.put("Wrist Idle", new SetWristIdleMode());
        eventMap.put("Wrist Position", new SetWristPositionMode(0));

        autoBuilder = new SwerveAutoBuilder(
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
    }

    public static SwerveAutoBuilder getAutoBuilder() {
        return autoBuilder;
    }  
}

