// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

	public enum ClawState {
		IDLE,
		PASSIVE,
		// if the necessary sequence for intaking is different
		CONE,
		CUBE,
		// if the necessary sequence for intaking is the same
		INTAKE
	}

	// public Spark motor = new Spark(0);
	// public Spark motorTwo = new Spark(1);

	public ClawState clawState = ClawState.IDLE;

	public CANSparkMax leftWheels = new CANSparkMax(Constants.Ports.CLAW_LEFT_WHEELS, MotorType.kBrushless);
	public CANSparkMax rightWheels = new CANSparkMax(Constants.Ports.CLAW_RIGHT_WHEELS, MotorType.kBrushless);
	public Solenoid snapper = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.CLAW_SOLENOID);
	public DigitalInput beamTrigger = new DigitalInput(Constants.Ports.CLAW_BEAM_BREAK);

	public Claw() {
		leftWheels.setIdleMode(IdleMode.kBrake);
		rightWheels.setIdleMode(IdleMode.kBrake);
	}

	public void setState(ClawState state) {
		clawState = state;
	}

	public ClawState getState() {
		return clawState;
	}

	public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
		// leftWheels.set(leftSpee%d);
		// rightWheels.set(rightSpeed);
		// motor.set(leftSpeed);
		// motorTwo.set(rightSpeed);

	}

	public void setSnapper(boolean isReleased) {
		snapper.set(isReleased);
	}

	public boolean getBeam() {
		return beamTrigger.get();
	}

	@Override
	public void periodic() {

		switch (clawState) {
			case IDLE:
				setWheelSpeeds(0, 0);
				break;
			case PASSIVE:
				setWheelSpeeds(-0.05, 0.05);
				break;
			case CONE:
				break;
			case CUBE:
				break;
			case INTAKE:
				break;
		}
	}
}
