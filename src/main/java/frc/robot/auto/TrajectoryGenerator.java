package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;

public class TrajectoryGenerator {
	public static TrajectoryGenerator INSTANCE = new TrajectoryGenerator();

	public static TrajectoryConfig configFast = new TrajectoryConfig(
		Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND,
		Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
		.setKinematics(Constants.Swerve.SWERVE_KINEMATICS);

	public static TrajectoryConfig configSlow = new TrajectoryConfig(
		Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND - 2,
		Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 2.5)
		.setKinematics(Constants.Swerve.SWERVE_KINEMATICS);

	public TrajectoryGenerator() {}

	public static Trajectory getstartToGamePiece() {
		return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
				List.of(
						new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
						new Pose2d(new Translation2d(-3, 0), Rotation2d.fromDegrees(0))),
				configSlow);
	}

	// public static Trajectory getGamePieceToStart() {
	// 	return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
	// 			List.of(
	// 					new Pose2d(new Translation2d(2, 0.5),
	// 							Rotation2d.fromDegrees(180)),
	// 					new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180))),
	// 			configFast);
	// }

	// public static Trajectory getTopTwoPieceStartToGamePiece() {
	// 	return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
	// 			List.of(
	// 					new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180)),
	// 					new Pose2d(new Translation2d(Units.inchesToMeters(225.0), Units.inchesToMeters(15)), Rotation2d.fromDegrees(180))),
	// 			configSlow);
	// }

	// public static Trajectory getTopTwoPieceGamePieceToShelf() {
	// 	return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
	// 			List.of(
	// 					new Pose2d(new Translation2d(Units.inchesToMeters(225), Units.inchesToMeters(15)), Rotation2d.fromDegrees(0)),
	// 					new Pose2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(20)), Rotation2d.fromDegrees(0))),
	// 			configSlow);
	// }
}
