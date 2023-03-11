package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase{
    public int moduleNumber;
    private double angleOffset;
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder cancoder;
    private SparkMaxPIDController driveController;
    private SparkMaxPIDController integratedAngleController;
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS,
            Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        integratedAngleController = angleMotor.getPIDController();
      
        /* Angle Encoder Config */
        cancoder = new CANCoder(moduleConstants.cancoderID);
        cancoder.configMagnetOffset(moduleConstants.angleOffset);
        
        configureDevices();

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
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity * 2.8, CANSparkMax.ControlType.kVelocity, 0, feedforward.calculate(velocity));
        }

        double angle = Math.abs(desiredState.speedMetersPerSecond) <= Constants.Swerve.MAX_SPEED * 0.01
        ? lastAngle
        : desiredState.angle.getDegrees();

        // double angle = desiredState.angle.getDegrees();
        // System.out.println(angleOutput + "   hdgjhsd   " + moduleNumber);
        // angleOutput = Math.abs(angleOutput) < .3 ? 0 : angleOutput;
        // angleOutput *= angleEncoder.getAbsolutePosition() < 180 ? 1 : -1;
        // angleMotor.set((wheelDegreesToNeo(angle) - integratedAngleEncoder.getPosition())  * -.0004);
    


        // System.out.println(neoToWheelDegrees(integratedAngleEncoder.getPosition()) + "   e      " + moduleNumber);
    //    System.out.println(wheelDegree);
        // System.out.println((angle + "    w    " + moduleNumber));

        // System.out.println(angleEncoder.getAbsolutePosition() + "      e       " + moduleNumber);
        // System.out.println(angle + "      e       " + moduleNumber);
        // System.out.println(desiredState.speedMetersPerSecond + "      e       " + moduleNumber);
    
  
        // if (moduleNumber == 3) {
            // angleMotor.set(angleController.calculate(angleEncoder.getAbsolutePosition(), angle));
            // System.out.println(angleMotor.get());
        // }
        // else {
        //     angleMotor.set(angleController.calculate(angleEncoder.getAbsolutePosition(), angle));
        // }

        integratedAngleController.setReference(angle, CANSparkMax.ControlType.kPosition);

        lastAngle = angle;
    }

    private void configureDevices() {
        integratedAngleEncoder.setPosition(cancoder.getAbsolutePosition() - angleOffset);

    // Drive motor configuration.
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
    // if (moduleNumber == 0) {
    //     driveMotor.setInverted(!Constants.Swerve.DRIVE_MOTOR_INVERT);
    // }
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setOpenLoopRampRate(Constants.Swerve.DRIVE_OPEN_LOOP_RAMP);
    driveMotor.setClosedLoopRampRate(Constants.Swerve.DRIVE_CLOSED_LOOP_RAMP);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_MOTOR_SMART_CURRENT);
 
    driveController.setP(Constants.Swerve.DRIVE_MOTOR_KP);
    driveController.setI(Constants.Swerve.DRIVE_MOTOR_KI);
    driveController.setD(Constants.Swerve.DRIVE_MOTOR_KD);
    driveController.setFF(Constants.Swerve.DRIVE_MOTOR_KF);
 
    driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_MOTOR_POSITION_CONVERSION);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_MOTOR_VELOCITY_CONVERSION);
    driveEncoder.setPosition(0);

    // Angle motor configuration.
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_MOTOR_SMART_CURRENT);

    integratedAngleController.setP(.02);
    integratedAngleController.setI(0);
    integratedAngleController.setD(0);

    integratedAngleController.setPositionPIDWrappingEnabled(true);
    integratedAngleController.setPositionPIDWrappingMaxInput(360);
    integratedAngleController.setPositionPIDWrappingMinInput(0);

    integratedAngleEncoder.setPositionConversionFactor(360 / Constants.Swerve.ANGLE_GEAR_RATIO);
    
    // CanCoder configuration.
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = Constants.Swerve.CANCONDER_INVERT;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

    cancoder.configAllSettings(canCoderConfiguration);
  }


    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = driveEncoder.getPosition(); 
        Rotation2d angle = Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        return new SwerveModulePosition(position, angle);
    }

    public double neoRotationsToWheelMeters(double neoRot) {
            double wheelRot = neoRot / Constants.Swerve.DRIVE_GEAR_RATIO;
            double wheelMeters = wheelRot * Constants.Swerve.WHEEL_CIRCUMFERENCE;
            return wheelMeters;
    }

    public double neoRPMToWheelMPS(double neoRPM) {
        double wheelRPM = neoRPM / Constants.Swerve.DRIVE_GEAR_RATIO;
        double wheelMPM = wheelRPM * Constants.Swerve.WHEEL_CIRCUMFERENCE;
        double wheelMPS = wheelMPM / 60;
        return wheelMPS;
    }

    public double neoToWheelDegrees(double neoTicks) {
        return neoTicks * 360 / Constants.Swerve.ANGLE_GEAR_RATIO;
    }

    public double wheelDegreesToNeo(double wheelDegrees) {
        return (wheelDegrees * Constants.Swerve.ANGLE_GEAR_RATIO) / 360;
    }

    // Used for Charge Station auto
    public void lockWheels() {
        integratedAngleController.setReference(90, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // if (moduleNumber == 0) {
        // System.out.println(getPosition());
        // }
  }
}