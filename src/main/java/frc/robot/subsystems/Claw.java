// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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

	public CANSparkMax leftWheels = new CANSparkMax(Constants.Ports.CLAW_LEFT_WHEELS, MotorType.kBrushless);
	public CANSparkMax rightWheels = new CANSparkMax(Constants.Ports.CLAW_RIGHT_WHEELS, MotorType.kBrushless);
	public TalonFX claw = new TalonFX(Constants.Ports.CLAW);
	public PneumaticHub pH = new PneumaticHub(Constants.Ports.PH_CAN_ID);
	public Solenoid snapper = pH.makeSolenoid(Constants.Ports.CLAW_SOLENOID);

	public Timer timer = new Timer();

	public boolean deactivateIntake = false;

	public Claw() {
		pH.enableCompressorAnalog(100, 120);
		leftWheels.setIdleMode(IdleMode.kBrake);
		rightWheels.setIdleMode(IdleMode.kBrake);
		claw.setNeutralMode(NeutralMode.Brake);

		leftWheels.setInverted(false);
		rightWheels.setInverted(true);
		claw.setInverted(true);

		timer.reset();
	}

	public void setState(ClawState state) {
		clawState = state;
	}

	public ClawState getState() {
		return clawState;
	}

	public void setWheelSpeeds(double speed) {
		leftWheels.set(speed);
		rightWheels.set(speed);
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

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Claw Current", claw.getStatorCurrent());

		// System.out.println(getState());
		// System.out.println(isIntakeDeactivated());

		if (RobotContainer.wrist.getWristPosition() > 70) {
			setSnapper(false);
		}
		else {
			setSnapper(true);
		}

		switch (clawState) {
			case IDLE:
				timer.stop();
				timer.reset();
				setWheelSpeeds(0);
				deactivateIntake = false;
				break;
			case PASSIVE:
				deactivateIntake = true;
				setWheelSpeeds(0.05);
				break;
			case INTAKE:
				timer.start();
				if (timer.get() > 0.3 && claw.getStatorCurrent() >= 25 && !deactivateIntake) {
					deactivateIntake = true;
					timer.stop();
					timer.reset();
					timer.start();
				} else if (!deactivateIntake) {
					setWheelSpeeds(0.4);
				}
				if (deactivateIntake && timer.get() > 0.2) {
					setWheelSpeeds(0);
					timer.stop();
					timer.reset();
					setState(ClawState.PASSIVE);
				}
				break;
			case RELEASE:
				deactivateIntake = false;
				setWheelSpeeds(-0.2);
		}
	}
}
