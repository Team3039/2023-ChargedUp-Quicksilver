package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Drive extends SubsystemBase {

    public SwerveModule[] swerveMods;
    public Pigeon2 gyro = new Pigeon2(4);
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swervePoseEstimator;

    public boolean isHighGear = false;

    public double[] previousPose = new double[2];

    public Drive() {
        setGyro(0);
        gyro.configMountPoseRoll(-3.4);

        swerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, getYaw(), getPositions());

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_KINEMATICS, getYaw(), getPositions(), new Pose2d());
        isHighGear = false;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        isHighGear ? MathUtil.clamp(translation.getX(), -1.0, 1.0) : translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setGear(boolean isHighGear) {
        this.isHighGear = isHighGear;
    }

    public boolean getGear() {
        return isHighGear;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    // Field Centric
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public static Supplier<Rotation2d> getSwerveHeadingSupplier(double theta) {
        return () -> Rotation2d.fromDegrees(theta);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = swerveMods[i].getPosition();
        }
        return positions;
    }

    public void setGyro(double degrees) {
        gyro.setYaw(degrees);
    }

    // Robot Centric
    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]);
    }

    public double getAngle() {
        return gyro.getYaw() % 360;
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    // Auto
    public void lockModules() {
        for (SwerveModule mod : swerveMods) {
            mod.lockWheels();
        }
    }

    // public Command moveToPose(Supplier<Pose2d> poseSupplier, Translation2d newOffset) {
    //     PIDController yaxisPid = new PIDController(
    //         Constants.Vision.Y_P, 
    //         Constants.Vision.Y_I, 
    //         Constants.Vision.Y_D
    //     );

    //     PIDController xaxisPid = new PIDController(
    //         Constants.Vision.X_P, 
    //         Constants.Vision.X_I, 
    //         Constants.Vision.X_D
    //     );

    //     PIDController thetaPid = new PIDController(
    //         Constants.Vision.THETA_P, 
    //         Constants.Vision.THETA_I, 
    //         Constants.Vision.THETA_D
    //     );

    //     thetaPid.enableContinuousInput(0, 2 * Math.PI);

    //     xaxisPid.setTolerance(Constants.Vision.X_TOLLERENCE);
    //     yaxisPid.setTolerance(Constants.Vision.Y_TOLLERENCE);
    //     thetaPid.setTolerance(Constants.Vision.THETA_TOLLERENCE);

    //     return runOnce(() -> {
    //         xaxisPid.calculate(swervePoseEstimator.getEstimatedPosition().getX());
    //         yaxisPid.calculate(swervePoseEstimator.getEstimatedPosition().getY());
    //         thetaPid.calculate(
    //                     swervePoseEstimator.getEstimatedPosition().getRotation().getRadians()
    //         );

    //         SmartDashboard.putNumber("In Auto Align", 1);
    //         Translation2d offset = newOffset;
    //         // Give offset a default value
    //         if (offset == null) {
    //             offset = 1;
    //         }

    //         // Get forward vector of pose and add it to offset
    //         Pose2d pose = poseSupplier.get();
    //         Rotation2d targetRot = pose.getRotation();
    //         offset = offset.rotateBy(targetRot);
    //         Translation2d targetTrans = pose.getTranslation();
    //         Translation2d offsetTarget = targetTrans.plus(offset);
    //         field.getObject("target").setPose(new Pose2d(offsetTarget, targetRot));

    //         // Set pid setpoints
    //         xaxisPid.setSetpoint(offsetTarget.getX());
    //         yaxisPid.setSetpoint(offsetTarget.getY());

    //         // Invert theta to ensure we're facing towards the target
    //         thetaPid.setSetpoint(targetRot.minus(kAutoAlign.DEFAULT_ROTATION).getRadians());
    //     }).andThen(run(
    //         () -> {
    //             SmartDashboard.putNumber("x tolerance", xaxisPid.getPositionError());
    //             SmartDashboard.putNumber("y tolerance", yaxisPid.getPositionError());
    //             SmartDashboard.putNumber("theta tolerance", thetaPid.getPositionError());

    //             drive(
    //                 new Translation2d(
    //                     xaxisPid.calculate(swerveOdometry.getEstimatedPosition().getX()),
    //                     yaxisPid.calculate(swerveOdometry.getEstimatedPosition().getY())
    //                 ),
    //                 thetaPid.calculate(
    //                     swerveOdometry.getEstimatedPosition().getRotation().getRadians()
    //                 ),
    //                 true, 
    //                 false
    //             );
    //         }
    //     )).until(
    //         () -> xaxisPid.atSetpoint() && yaxisPid.atSetpoint() && thetaPid.atSetpoint()
    //     ).andThen(
    //         () -> { 
    //             SmartDashboard.putNumber("In Auto Align", 0);

    //             xaxisPid.close(); 
    //             yaxisPid.close(); 
    //             thetaPid.close(); 
    //         }
    //     );
    // }

    @Override
    public void periodic() {
        // System.out.println(getStates()[0]);
        // System.out.println(getAngle());
        // System.out.println(getRoll() + " Roll");
        // System.out.println(getPitch() + " Pitch");

        // System.out.println(swerveOdometry.getPoseMeters());  
        // System.out.println(getRoll());     
        System.out.println(getAngle());  

        previousPose[0] = swerveOdometry.getPoseMeters().getX();
        previousPose[1] = swerveOdometry.getPoseMeters().getY();
        swerveOdometry.update(getYaw(), getPositions());
        swervePoseEstimator.update(getYaw(), getPositions());
        

        SmartDashboard.putNumber("Pigeon Reading", gyro.getYaw());
        SmartDashboard.putNumber("Odometry X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", swerveOdometry.getPoseMeters().getY());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}