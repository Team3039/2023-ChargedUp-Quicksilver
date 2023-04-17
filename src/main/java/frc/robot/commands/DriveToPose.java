package frc.robot.commands;

import static frc.robot.Constants.Vision.FIELD_WIDTH_METERS;
import static frc.robot.Constants.Vision.THETA_D;
import static frc.robot.Constants.Vision.THETA_I;
import static frc.robot.Constants.Vision.THETA_P;
import static frc.robot.Constants.Vision.X_D;
import static frc.robot.Constants.Vision.X_I;
import static frc.robot.Constants.Vision.X_P;
import static frc.robot.Constants.Vision.Y_D;
import static frc.robot.Constants.Vision.Y_I;
import static frc.robot.Constants.Vision.Y_P;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;


/**
 * Command to drive to a pose.
 */
public class DriveToPose extends CommandBase {
  
  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      3,
      5);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      3 * 0.4,
      5);

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private final Drive swerve;
  private final Pose2d initialPose;
  private final Pose2d goalPose;
  private final boolean useAllianceColor;

  public DriveToPose (Drive swerve, Pose2d goalPose, boolean useAllianceColor) {
    this.swerve = swerve;
    this.initialPose = swerve.getPoseEstimate();
    this.goalPose = goalPose;
    this.useAllianceColor = useAllianceColor;

    xController = new PIDController(X_P, X_I, X_D);
    yController = new PIDController(Y_P, Y_I, Y_D);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new PIDController(THETA_P, THETA_I, THETA_D);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(swerve);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;
    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    } 
  }

  public boolean atGoal() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }

  private void resetPIDControllers() {
    var robotPose = initialPose;
    thetaController.reset();
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    var robotPose = swerve.getPoseEstimate();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX(), goalPose.getX());
    if (xController.atSetpoint()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY(), goalPose.getY());
    if (yController.atSetpoint()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians(), goalPose.getRotation().getRadians());
    if (thetaController.atSetpoint()) {
      omegaSpeed = 0;
    }

    Translation2d translation = new Translation2d(xSpeed, ySpeed);
    swerve.drive(translation, omegaSpeed, true, false);
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, false);  
  }

}