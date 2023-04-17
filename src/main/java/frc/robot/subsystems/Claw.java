// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Claw extends SubsystemBase {

	public enum ClawState {
		IDLE,
		PASSIVE,
		INTAKE,
		RELEASE
	}

	// public Spark motor = new Spark(0);
	// public Spark motorTwo = new Spark(1);

	public ClawState clawState = ClawState.IDLE;

	// public CANSparkMax leftWheels = new CANSparkMax(Constants.Ports.CLAW_LEFT_WHEELS, MotorType.kBrushless);
	// public CANSparkMax rightWheels = new CANSparkMax(Constants.Ports.CLAW_RIGHT_WHEELS, MotorType.kBrushless);
	public TalonFX claw = new TalonFX(Constants.Ports.CLAW);
	public PneumaticHub pH = new PneumaticHub(Constants.Ports.PH_CAN_ID);
	public Solenoid snapper = pH.makeSolenoid(Constants.Ports.CLAW_SOLENOID);

	public Timer timer = new Timer();

	public boolean deactivateIntake = false;

	public boolean allowSnapping = false;

	public Claw() {
		// pH.disableCompressor();
		// leftWheels.setIdleMode(IdleMode.kBrake);
		// rightWheels.setIdleMode(IdleMode.kBrake);
		claw.setNeutralMode(NeutralMode.Brake);

		// leftWheels.setInverted(false);
		// rightWheels.setInverted(true);
		claw.setInverted(true);

		timer.reset();

		claw.setStatusFramePeriod(255, 0);
	}

	public void setState(ClawState state) {
		clawState = state;
	}

	public ClawState getState() {
		return clawState;
	}

	public void setWheelSpeed(double speed) {
		claw.set(ControlMode.PercentOutput, speed);
	}

	public void setSnapper(boolean isReleased) {
		snapper.set(isReleased);
	}

	public boolean getSnapper() {
		return snapper.get();
	}

	public boolean isIntakeDeactivated() {
		return deactivateIntake;
	}

	public void isSnappingAllowed(boolean isAllowed) {
		allowSnapping = isAllowed;
	}

	@Override
	public void periodic() {
		if (pH.getPressureSwitch()) {
			pH.disableCompressor();
		}
		else {
			pH.enableCompressorAnalog(100, 120);
		}
		// SmartDashboard.putNumber("Claw Current", claw.getStatorCurrent());
		SmartDashboard.putBoolean("Is Intake Deactivated", isIntakeDeactivated());
		SmartDashboard.putString("Claw State", String.valueOf(getState()));



		if (!allowSnapping) {
			if (RobotContainer.wrist.getWristPosition() > 75 && RobotContainer.elevator.getState().equals(ElevatorState.IDLE)) {
				setSnapper(false);
			}
			else {
			setSnapper(true);
			}
		}

		switch (clawState) {
			case IDLE:
				timer.stop();
				timer.reset();
				setWheelSpeed(0);
				deactivateIntake = false;
				break;
			case PASSIVE:
				deactivateIntake = true;
				setWheelSpeed(0.08);
				break;
			case INTAKE:
				timer.start();
				if (timer.get() > 0.5 && claw.getStatorCurrent() > 9.0 && allowSnapping) {
					setSnapper(false);
					new WaitCommand(0.3);
					Wrist.setSetpoint(9.3);
				}
				// if (allowSnapping) {
				// 	setSnapper(false);
				// }
				if (timer.get() > 0.5 && claw.getStatorCurrent() >= 50 && !deactivateIntake) {
					deactivateIntake = true;
					timer.stop();
					timer.reset();
					timer.start();
				} else if (!deactivateIntake) {
					setWheelSpeed(0.8);
				}
				if (deactivateIntake && timer.get() > 0.1) {
					setWheelSpeed(0);
					Wrist.setSetpoint(20);
					timer.stop();
					timer.reset();
				}
				break;
			case RELEASE:
				deactivateIntake = false;
				setWheelSpeed(-0.8);
		}
	}
}
