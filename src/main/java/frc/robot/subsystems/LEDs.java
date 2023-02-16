// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

    public final int LEDcount = 36;

    public AddressableLED leds = new AddressableLED(8);
    public AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDcount);

	public LEDs() {
        leds.setLength(LEDcount);
        leds.setData(buffer);
        leds.start();
	}

    public void setColorRGB(int r, int g, int b) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
         }
         leds.setData(buffer);
    }

	@Override
	public void periodic() {
		switch(ledState) {
			case IDLE:
       
			  break;
			case CONE:
              setColorRGB(200, 200, 0);
			  break;
			case CUBE:
              setColorRGB(200, 0, 200);
			  break;
		}

}
}
