// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {

	public enum LEDState {
		IDLE,
		CUBE,
		CONE
	}

	public LEDState ledState = LEDState.IDLE;

	public PowerDistribution pDH = new PowerDistribution(9, ModuleType.kRev);

	public final int LEDcount = 50;

	// private int rainbowStart = 0;

	public boolean desiresCone = false;
	public boolean desiresCube = false;

	public AddressableLED leds = new AddressableLED(9);
	public AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDcount);

	public LEDs() {
		leds.setLength(LEDcount);
		leds.setData(buffer);
		leds.start();
	}

	public LEDState getState() {
		return ledState;
	}

	public void setState(LEDState state) {
		ledState = state;
	}

	public void setColorRGB(int r, int g, int b) {
		for (var i = 0; i < buffer.getLength(); i++) {
			buffer.setRGB(i, r, g, b);

		}
		leds.setData(buffer);
	}

	// private void rainbow() {
	// // For every pixel
	// for (var i = 0; i < buffer.getLength(); i++) {
	// // Calculate the hue - hue is easier for rainbows because the color
	// // shape is a circle so only one value needs to precess
	// final var hue = (rainbowStart + (i * 180 / buffer.getLength())) % 180;
	// // Set the value
	// buffer.setHSV(i, hue, 255, 128);
	// }
	// // Increase by to make the rainbow "move"
	// rainbowStart += 3;
	// // Check bounds
	// rainbowStart %= 180;
	// }

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Desires Cube", desiresCube);
		SmartDashboard.putBoolean("Desires Cone", desiresCone);


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
		switch (ledState) {
			case IDLE:
				if (DriverStation.isDisabled()) {
					// rainbow();
				} else {
					setColorRGB(0, 0, 0);
				}
				desiresCone = false;
				desiresCube = false;
				break;
			case CONE:
				setColorRGB(50, 50, 0);
				desiresCone = true;
				desiresCube = false;
				break;
			case CUBE:
				setColorRGB(50, 0, 50);
				desiresCone = false;
				desiresCube = true;
				break;
		}

	}
}
