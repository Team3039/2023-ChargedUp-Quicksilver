package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CTREModuleState;
import frc.lib.Conversions;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{
    public int moduleNumber;
    private double angleOffset;
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;
    private SparkMaxPIDController driveController;
    private SparkMaxPIDController integratedAngleController;
    private PIDController angleController;
    private double angleOutput;
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS,
            Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        
        configAngleEncoder();
        angleEncoder.configMagnetOffset(moduleConstants.angleOffset);

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
            Constants.Swerve.ANGLE_MOTOR_KD);
        configAngleController();

        integratedAngleEncoder = angleMotor.getEncoder();
        resetToAbsolute();
        integratedAngleEncoder.setPositionConversionFactor(1);
        integratedAngleController = angleMotor.getPIDController();
        integratedAngleController.setP(-.02);
        integratedAngleController.setI(0);
        integratedAngleController.setD(0);
        // integratedAngleController.setPositionPIDWrappingMinInput(0);
        // integratedAngleController.setPositionPIDWrappingMaxInput(21.42857142857);
        integratedAngleController.setOutputRange(Constants.Swerve.DRIVE_MOTOR_MIN_OUTPUT, Constants.Swerve.DRIVE_MOTOR_MAX_OUTPUT);

        lastAngle = getState().angle.getDegrees();

        angleMotor.burnFlash();
        driveMotor.burnFlash();

        };
    

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // System.out.println(desiredState.angle.getDegrees() + "     e     " + moduleNumber );
        desiredState = CTREModuleState.optimize(desiredState, getState().angle );

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity, ControlType.kVelocity, 0, 
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        // double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ? lastAngle
        //         : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.

        double angle = desiredState.angle.getDegrees();
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
        angleMotor.set(angleController.calculate(angleEncoder.getAbsolutePosition(), angle));
        lastAngle = angle;
    }


    private void resetToAbsolute() {
        double absolutePosition = wheelDegreesToNeo(getCanCoder().getDegrees() - angleOffset);
        integratedAngleEncoder.setPosition(wheelDegreesToNeo(angleEncoder.getAbsolutePosition()));
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_MOTOR_SMART_CURRENT);
        angleMotor.setSecondaryCurrentLimit(Constants.Swerve.ANGLE_MOTOR_SECONDARY_LIMIT);
        angleMotor.setIdleMode(IdleMode.kCoast);

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
        driveEncoder.setPosition(0);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_MOTOR_POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_MOTOR_VELOCITY_CONVERSION);
    }

    private void configDriveController() {
        driveController.setP(Constants.Swerve.DRIVE_MOTOR_KP);
        driveController.setI(Constants.Swerve.DRIVE_MOTOR_KI);
        driveController.setD(Constants.Swerve.DRIVE_MOTOR_KD);
        driveController.setFF(Constants.Swerve.DRIVE_MOTOR_KF);
        driveController.setOutputRange(Constants.Swerve.DRIVE_MOTOR_MIN_OUTPUT, Constants.Swerve.DRIVE_MOTOR_MAX_OUTPUT);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleEncoder.configSensorDirection(Constants.Swerve.CANCONDER_INVERT);
        angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        angleEncoder.configFeedbackCoefficient(.087890625, "Degrees", SensorTimeBase.PerSecond);
        angleEncoder.setPosition(angleEncoder.getAbsolutePosition());
    }

    private void configAngleController() {
        angleController.enableContinuousInput(0, 360);
    }


    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(angleEncoder.getPosition());
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = driveEncoder.getPosition(); 
        Rotation2d angle = Rotation2d.fromDegrees(angleEncoder.getPosition());
        return new SwerveModulePosition(position, angle);
    }

    public double neoToWheelDegrees(double neoTicks) {
        return neoTicks * 360 / 21.42857142857 ;
    }

    public double wheelDegreesToNeo(double wheelDegrees) {
        return (wheelDegrees * Constants.Swerve.ANGLE_GEAR_RATIO) / 360;
    }

    @Override
    public void periodic() {
        // System.out.println(integratedAngleEncoder.getPosition() + "   v  gh      " + moduleNumber);
    // integratedAngleEncoder.setPosition(integratedAngleEncoder.getPosition() >= 0 ? 
    //                                    integratedAngleEncoder.getPosition() % 21.42857142857 :
    //                                    (21.42857142857 - Math.abs(integratedAngleEncoder.getPosition())) % 21.42857142857);
    // angleController.setSetpoint(lastAngle)
    // if (!angleController.atSetpoint()) {
    
    // }
    // else {
    // angleOutput = 0;
    // }

    
  }
}