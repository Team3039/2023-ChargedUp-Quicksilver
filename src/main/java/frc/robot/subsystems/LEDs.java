// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {

	public DigitalOutput[] outputs = {
		new DigitalOutput(Constants.Ports.LED_OUTPUT_A),
		new DigitalOutput(Constants.Ports.LED_OUTPUT_B),
		new DigitalOutput(Constants.Ports.LED_OUTPUT_C)
	};

	// White Fade. Active while booting up (Not referenced in code) 
	private boolean[] whiteFade = {false, false, false};

	// Turns LEDs Off. Active during teleop and autonomous when no other states are active
	private boolean[] ledsOff = {true, true, true};

	// Fire Code in red. Active when disabled while on the red alliance
	private boolean[] redFire = {false, true, false};
	
	// Fire Code in blue. Active when disabled while on the blue alliance
	private boolean[] blueFire = {false, true, true};

	// Yellow Fade. Active in teleop when driver signals that they want a Cone
	private boolean[] yellowFade = {true, false, false};

	// Purple Fade. Active in teleop when driver signals that they want a Cube
	private boolean[] purpleFade = {true, true, false};

	private boolean[] states;

	public PowerDistribution pDH = new PowerDistribution(9, ModuleType.kRev);

	public boolean desiresCone = false;
	public boolean desiresCube = false;

	public LEDs() {}

	public void setDesiredPiece(boolean cone, boolean cube) {
		desiresCone = cone;
		desiresCube = cube;
	}

	public boolean getDesiresCone() {
		return desiresCone;
	}
	
	public boolean getDesiresCube() {
		return desiresCube;
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Desires Cube", desiresCube);
		SmartDashboard.putBoolean("Desires Cone", desiresCone);

		// System.out.println("" + outputs[0].get() + "  " + outputs[1].get() + "  " + outputs[2].get());
		
		if (DriverStation.isTeleopEnabled()) {
			if (RobotContainer.claw.isIntakeDeactivated()) {
				pDH.setSwitchableChannel(true);
			}
			else {
				pDH.setSwitchableChannel(false);
			}
		}
		else {
			pDH.setSwitchableChannel(true);
		}
		
		if (DriverStation.isDisabled()) {
			if (DriverStation.getAlliance().equals(Alliance.Red)) {
				states = redFire;
			}
			else {
				states = blueFire;
			}
		}
		else {
			if (desiresCone == true) {
				states = yellowFade;
			}
			else if (desiresCube == true) {
				states = purpleFade;
			}
			else {
				states = ledsOff;
			}
		}
		for (int i = 0; i < 3; i++) {
			outputs[i].set(states[i]);
		}
	}
}
