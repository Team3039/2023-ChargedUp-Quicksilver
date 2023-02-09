// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/** Add your docs here. */
public class PPTrajectoryGenerator {
    public static PPTrajectoryGenerator INSTANCE = new PPTrajectoryGenerator();

    public static List<PathPlannerTrajectory> getTestPath() {
            return PathPlanner.loadPathGroup("Test Code", 
                   new PathConstraints(1.0, 3));
    }


}
