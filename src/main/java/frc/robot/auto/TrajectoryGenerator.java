package frc.robot.auto;

import java.util.List;

// import javax.management.MalformedObjectNameException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class TrajectoryGenerator {
        public static TrajectoryGenerator INSTANCE = new TrajectoryGenerator();

        public static TrajectoryConfig configFast = new TrajectoryConfig(
                        Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND - 0.5,
                        Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 1.5)
                                        .setKinematics(Constants.Swerve.SWERVE_KINEMATICS);
        // .setReversed(true);

        public static TrajectoryConfig configSlow = new TrajectoryConfig(
                        Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND - 2,
                        Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 2.5)
                                        .setKinematics(Constants.Swerve.SWERVE_KINEMATICS);
        // .setReversed(true);
        
        public static Trajectory getstartToGamePiece() {
                return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                        List.of(
                                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                                new Pose2d(new Translation2d(Units.inchesToMeters(39.37), Units.inchesToMeters(0)),
                                        Rotation2d.fromDegrees(0))),
                        configFast);

                        
        }

        public static Trajectory getGamePieceToStart() {
                return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                        List.of(
                                new Pose2d(new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(0)),
                                        Rotation2d.fromDegrees(0)),
                                new Pose2d()),
                        configFast);
        }
}