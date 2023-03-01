package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
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

    @Override
    public void periodic() {
        // System.out.println(getStates()[0]);
        // System.out.println(getAngle());
        // System.out.println(getRoll() + " Roll");
        // System.out.println(getPitch() + " Pitch");

        System.out.println(swerveOdometry.getPoseMeters());         

        previousPose[0] = swerveOdometry.getPoseMeters().getX();
        previousPose[1] = swerveOdometry.getPoseMeters().getY();
        swerveOdometry.update(getYaw(), getPositions());

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