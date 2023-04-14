// package frc.robot.commands;

// import static frc.robot.Constants.Vision.THETA_D;
// import static frc.robot.Constants.Vision.THETA_I;
// import static frc.robot.Constants.Vision.THETA_P;
// import static frc.robot.Constants.Vision.X_D;
// import static frc.robot.Constants.Vision.X_I;
// import static frc.robot.Constants.Vision.X_P;
// import static frc.robot.Constants.Vision.Y_D;
// import static frc.robot.Constants.Vision.Y_I;
// import static frc.robot.Constants.Vision.Y_P;
// import static frc.robot.Constants.Vision.FIELD_WIDTH_METERS;

// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.subsystems.drive;

// /**
//  * Command to drive to a pose.
//  */
// public class DriveToPose extends CommandBase {
  
//   private static final double TRANSLATION_TOLERANCE = 0.02;
//   private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

//   /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
//   private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
//       3,
//       5);
//   private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
//       3 * 0.4,
//       5);

//   private final ProfiledPIDController xController;
//   private final ProfiledPIDController yController;
//   private final ProfiledPIDController thetaController;

//   private final Drive drive;
//   private final Pose2d poseProvider;
//   private final Pose2d goalPose;
//   private final boolean useAllianceColor;

//   public DriveToPose (Drive drive, Pose2d poseProvider, Pose2d goalPose, boolean useAllianceColor) {
//     this.drive = drive;
//     this.poseProvider = poseProvider;
//     this.goalPose = goalPose;
//     this.useAllianceColor = useAllianceColor;

//     xController = new ProfiledPIDController(X_P, X_I, X_D, DEFAULT_XY_CONSTRAINTS);
//     yController = new ProfiledPIDController(Y_P, Y_I, Y_D, DEFAULT_XY_CONSTRAINTS);
//     xController.setTolerance(TRANSLATION_TOLERANCE);
//     yController.setTolerance(TRANSLATION_TOLERANCE);
//     thetaController = new ProfiledPIDController(THETA_P, THETA_I, THETA_D, omegaConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
//     thetaController.setTolerance(THETA_TOLERANCE);

//     addRequirements(drive);
//   }


//   @Override
//   public void initialize() {
//     resetPIDControllers();
//     var pose = goalPose;
//     if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
//       Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());
//       Rotation2d transformedHeading = pose.getRotation().times(-1);
//       pose = new Pose2d(transformedTranslation, transformedHeading);
//     }
//     thetaController.setGoal(pose.getRotation().getRadians());
//     xController.setGoal(pose.getX());
//     yController.setGoal(pose.getY());  }

//   public boolean atGoal() {
//     return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
//   }

//   private void resetPIDControllers() {
//     var robotPose = poseProvider.get();
//     thetaController.reset(robotPose.getRotation().getRadians());
//     xController.reset(robotPose.getX());
//     yController.reset(robotPose.getY());
//   }

//   @Override
//   public void execute() {
//     var robotPose = poseProvider.get();
//     // Drive to the goal
//     var xSpeed = xController.calculate(robotPose.getX());
//     if (xController.atGoal()) {
//       xSpeed = 0;
//     }

//     var ySpeed = yController.calculate(robotPose.getY());
//     if (yController.atGoal()) {
//       ySpeed = 0;
//     }

//     var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
//     if (thetaController.atGoal()) {
//       omegaSpeed = 0;
//     }

//     drive.drive(
//       ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
//   }

//   @Override
//   public boolean isFinished() {
//     return atGoal();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     swerve.drive(new Translation2d(), 0, true, false)  
//   }

// }