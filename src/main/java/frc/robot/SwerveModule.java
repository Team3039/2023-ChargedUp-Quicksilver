package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Conversions;
import frc.lib.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private CANCoder angleEncoder;
    private SparkMaxPIDController driveController;
    private PIDController angleController;
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS,
            Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();
    
        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        driveEncoder = driveMotor.getEncoder();
        configDriveEncoder();

        driveController = driveMotor.getPIDController();
        configDriveController();

        angleController = new PIDController(
            Constants.Swerve.ANGLE_MOTOR_KP, 
            Constants.Swerve.ANGLE_MOTOR_KI, 
            Constants.Swerve.ANGLE_MOTOR_KD,   
            Constants.Swerve.DRIVE_MOTOR_KF);
        configAngleController();
        lastAngle = getState().angle.getDegrees();

        angleMotor.burnFlash();
        driveMotor.burnFlash();

        };
    

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    // private void resetToAbsolute() {
    //     double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset,
    //             Constants.Swerve.ANGLE_GEAR_RATIO);
    //     angleMotor.setSelectedSensorPosition(absolutePosition);
    // }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_MOTOR_SMART_CURRENT);
        angleMotor.setSecondaryCurrentLimit(Constants.Swerve.ANGLE_MOTOR_SECONDARY_LIMIT);
        angleMotor.setIdleMode(IdleMode.kBrake);

    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        driveMotor.setClosedLoopRampRate(Constants.Swerve.DRIVE_CLOSED_LOOP_RAMP);
        driveMotor.setOpenLoopRampRate(Constants.Swerve.DRIVE_OPEN_LOOP_RAMP);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_MOTOR_SMART_CURRENT);
        driveMotor.setSecondaryCurrentLimit(Constants.Swerve.DRIVE_MOTOR_SECONDARY_LIMIT);
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    private void configDriveEncoder() {
        driveEncoder.setPositionConversionFactor(Constants.Swerve.WHEEL_CIRCUMFERENCE / Constants.Swerve.DRIVE_GEAR_RATIO);
    }

    private void configDriveController() {
        driveController.setP(Constants.Swerve.DRIVE_MOTOR_KP);
        driveController.setI(Constants.Swerve.DRIVE_MOTOR_KI);
        driveController.setD(Constants.Swerve.DRIVE_MOTOR_KD);
        driveController.setFF(Constants.Swerve.DRIVE_MOTOR_KF);
        driveController.setOutputRange(Constants.Swerve.DRIVE_MOTOR_MIN_OUTPUT, Constants.Swerve.DRIVE_MOTOR_MAX_OUTPUT)
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleEncoder.configSensorDirection(Constants.Swerve.CANCONDER_INVERT);
        angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        angleEncoder.configFeedbackCoefficient(.087890625, "Degrees", SensorTimeBase.PerSecond);
    }

    private void configAngleController() {

    }


    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(),
                Constants.Swerve.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
                Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(),
                Constants.Swerve.ANGLE_GEAR_RATIO));
        return new SwerveModulePosition(position, angle);
    }
}