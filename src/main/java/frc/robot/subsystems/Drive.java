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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

    // public static Drive INSTANCE = new Drive();

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    // private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();

    public static Trajectory trajectory = new Trajectory();
    public static TrapezoidProfile.Constraints thetaController;

    public boolean isHighGear = false;

    public double[] previousPose = new double[2];

    public Drive() {
        gyro = new Pigeon2(4);
        gyro.configFactoryDefault();
        setGyro(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, getYaw(), getPositions());

        thetaController = new TrapezoidProfile.Constraints(
                Constants.AutoConstants.K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                Constants.AutoConstants.K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        isHighGear = false;
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        isHighGear ? MathUtil.clamp(translation.getX(), -1.0, 1.0) : 
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            // System.out.println(swerveModuleStates[mod.moduleNumber].angle.getDegrees() + "      e     " + mod.moduleNumber);

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

        for (SwerveModule mod : mSwerveMods) {
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
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = mSwerveMods[i].getPosition();
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

    // public RigidTransform2 getPoseAtTime(double timestamp) {
    //     if (latencyCompensationMap.isEmpty()) {
    //         return RigidTransform2.ZERO;
    //     }
    //     return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
    // }

    @Override
    public void periodic() {
        previousPose[0] = swerveOdometry.getPoseMeters().getX();
        previousPose[1] = swerveOdometry.getPoseMeters().getY();
        swerveOdometry.update(getYaw(), getPositions());

        SmartDashboard.putNumber("Pigeon Reading", gyro.getYaw());
        SmartDashboard.putNumber("Odometry X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", swerveOdometry.getPoseMeters().getY());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
       }
    }
}