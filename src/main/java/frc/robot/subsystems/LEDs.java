// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {

	public enum LEDState {
		IDLE,
		CUBE,
		CONE
	}

	public LEDState ledState = LEDState.IDLE;

	DigitalOutput[] outputs = {
			new DigitalOutput(Constants.Ports.LED_OUTPUT_A),
			new DigitalOutput(Constants.Ports.LED_OUTPUT_B)
	};
	// Active when the robot is booting up (not referenced in code)
	// White Flash
	public boolean[] isBootingUp = { true, true };
	// Active when operator activates Cube Mode
	// Solid Purple
	public boolean[] desiresCube = { false, true };
	// Active when operator activates Cone Mode
	// Solid Yellow
	public boolean[] desiresCone = { true, false };
	// Active whenever the robot is on and does not meet the above cases
	// Fire Pattern.
	public boolean[] isIdle = { false, false };

	public boolean[] states;

	public LEDs() {
	}

	@Override
	public void periodic() {
		switch(ledState) {
			case IDLE:
			  states = isIdle;
			  break;
			case CONE:
				states = desiresCone;
			  break;
			case CUBE:
			  states = desiresCube;
			  break;
		}

		outputs[0].set(states[0]);
		outputs[1].set(states[1]);
}
}
