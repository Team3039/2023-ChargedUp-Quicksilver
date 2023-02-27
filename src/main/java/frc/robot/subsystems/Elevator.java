// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION
	}

	public ElevatorState elevatorState = ElevatorState.IDLE;

	// public CANSparkMax elevatorA = new CANSparkMax(Constants.Ports.ELEVATOR_A,
	// MotorType.kBrushless);
	public CANSparkMax elevator = new CANSparkMax(Constants.Ports.ELEVATOR_B, MotorType.kBrushless);

	public RelativeEncoder encoder = elevator.getEncoder();

	// public SparkMaxPIDController controller = elevator.getPIDController();

	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			Constants.Elevator.ELEVATOR_KS,
			Constants.Elevator.ELEVATOR_KG,
			Constants.Elevator.ELEVATOR_KV);

	private ProfiledPIDController profiledController = new ProfiledPIDController(
			Constants.Elevator.ELEVATOR_KP,
			Constants.Elevator.ELEVATOR_KI,
			Constants.Elevator.ELEVATOR_KD,
			new TrapezoidProfile.Constraints(
					Constants.Elevator.ELEVATOR_MAX_VEL,
					Constants.Elevator.ELEVATOR_MAX_ACCEL));

	private PIDController controller = new PIDController(
			Constants.Elevator.ELEVATOR_KP,
			Constants.Elevator.ELEVATOR_KI,
			Constants.Elevator.ELEVATOR_KD);

	SparkMaxLimitSwitch forwardLimit = elevator.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
	SparkMaxLimitSwitch ReverseLimit = elevator.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

	// rotations
	public static double setpointElevator = 0;

	public Elevator() {
		forwardLimit.enableLimitSwitch(false);
		ReverseLimit.enableLimitSwitch(false);
		// elevatorB.follow(elevatorA, true);

		// elevatorA.setIdleMode(IdleMode.kBrake);
		elevator.setIdleMode(IdleMode.kBrake);

		// elevatorA.setInverted(false);
		elevator.setInverted(false);

		// elevatorA.setSoftLimit(SoftLimitDirection.kForward, 10000);
		// elevatorA.setSoftLimit(SoftLimitDirection.kReverse, -1000);
		elevator.enableSoftLimit(SoftLimitDirection.kForward, true);
		elevator.enableSoftLimit(SoftLimitDirection.kReverse, true);
		elevator.setSoftLimit(SoftLimitDirection.kForward, 87);
		elevator.setSoftLimit(SoftLimitDirection.kReverse, 0);

		// elevatorA.burnFlash();
		elevator.burnFlash();
		
		controller.setTolerance(3);
		profiledController.setTolerance(3);
	}

	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	public ElevatorState getState() {
		return elevatorState;
	}

	public void setElevatorOpenLoop(double percent) {
		// if (!isAt;Limit(percent)) {
		elevator.set(percent);
		// elevator.set(percent + Constants.Elevator.ELEVATOR_KS);
		// }
	}

	public void setElevatorClosedLoop(boolean isProfiled) {
		double output = 0;
		if (isProfiled) {
			profiledController.setGoal(setpointElevator);
			output = profiledController.calculate(encoder.getPosition()) +
					feedForward.calculate(profiledController.getSetpoint().velocity);
			elevator.set(output);
		} else {
			output = controller.calculate(encoder.getPosition(), setpointElevator) + Constants.Elevator.ELEVATOR_KS;
			elevator.set(MathUtil.clamp(output, -.15, .3));
		}
	}

	// public boolean isAtHardLimit(double output) {
	// return (getLimitSwitch() && output > 0);
	// }

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	public boolean isAtSetpoint(boolean isProfiled) {
		return  Math.abs((setpointElevator - encoder.getPosition())) <= 3;
	  }

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
		SmartDashboard.putNumber("Elevator Output", elevator.get());
		// System.out.println(encoder.getPosition());
		// System.out.println(elevator.get());
		// System.out.println(isAtSetpoint(false));
		switch (elevatorState) {
			case IDLE:
				setSetpoint(0);
				setElevatorClosedLoop(false);
				break;
			case MANUAL:
				setElevatorOpenLoop(RobotContainer.operatorPad.getLeftY());
				break;
			case POSITION:
				setElevatorClosedLoop(false);
				break;
		}
	}
}
