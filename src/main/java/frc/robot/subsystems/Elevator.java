// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
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

	// public CANSparkMax elevatorA = new CANSparkMax(Constants.Ports.ELEVATOR_A, MotorType.kBrushless);
	public CANSparkMax elevatorB = new CANSparkMax(Constants.Ports.ELEVATOR_B, MotorType.kBrushless);

	public RelativeEncoder encoderA = elevatorB.getEncoder();

	public SparkMaxPIDController controllerA = elevatorB.getPIDController();

	// public DigitalInput limitSwitch = new DigitalInput(Constants.Ports.ELEVATOR_LIMIT_SWITCH);

	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			Constants.Elevator.ELEVATOR_KS,
			Constants.Elevator.ELEVATOR_KG,
			Constants.Elevator.ELEVATOR_KV);

	private ProfiledPIDController controller = new ProfiledPIDController(
			Constants.Elevator.ELEVATOR_KP,
			Constants.Elevator.ELEVATOR_KI,
			Constants.Elevator.ELEVATOR_KD,
			new TrapezoidProfile.Constraints(
					Constants.Elevator.ELEVATOR_MAX_VEL,
					Constants.Elevator.ELEVATOR_MAX_ACCEL));

	// rotations
	public static double setpointElevator = 0;

	public Elevator() {
		// elevatorB.follow(elevatorA, true);

		// elevatorA.setIdleMode(IdleMode.kBrake);
		elevatorB.setIdleMode(IdleMode.kBrake);

		// elevatorA.setInverted(false);
		elevatorB.setInverted(false);

		// elevatorA.setSoftLimit(SoftLimitDirection.kForward, 10000);
		// elevatorA.setSoftLimit(SoftLimitDirection.kReverse, -1000);
		elevatorB.setSoftLimit(SoftLimitDirection.kForward, 10000);
		elevatorB.setSoftLimit(SoftLimitDirection.kReverse, -1000);

		// controllerA.setP(Constants.Elevator.ELEVATOR_KP);
		// controllerA.setI(Constants.Elevator.ELEVATOR_KI);
		// controllerA.setD(Constants.Elevator.ELEVATOR_KD);

		// elevatorA.burnFlash();
		elevatorB.burnFlash();
	}

	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	public ElevatorState getState() {
		return elevatorState;
	}

	public void setElevatorOpenLoop(double percent) {
		// if (!isAtHardLimit(percent)) {
			// elevatorA.set(percent);
			elevatorB.set(percent);
		// }
	}

	public void setElevatorClosedLoop() {		
			controller.setGoal(setpointElevator);
			double output = controller.calculate(encoderA.getPosition()) +
					         feedForward.calculate(controller.getSetpoint().velocity);
			// if (!isAtHardLimit(output)) {
				elevatorB.set(output);
			// }
		}
	

	// public boolean isAtHardLimit(double output) {
	// 	return (getLimitSwitch() && output > 0);
	// }

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	// public boolean getLimitSwitch() {
	// 	// return limitSwitch.get();
	// }

	@Override
	public void periodic() {
		switch (elevatorState) {
			case IDLE:
				// setSetpoint(0);
				// setElevatorClosedLoop();
				break;
			case MANUAL:
				setElevatorOpenLoop(RobotContainer.operatorPad.getLeftY());
				break;
			case POSITION:
				setElevatorClosedLoop();
				break;
		}
	}
}
