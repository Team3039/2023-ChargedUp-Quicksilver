// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
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

    public final int LEDcount = 50;

    public AddressableLED leds = new AddressableLED(8);
    public AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDcount);

	public LEDs() {
        leds.setLength(LEDcount);
        leds.setData(buffer);
        leds.start();
	}

	public LEDState getState(){
		return ledState;
	}

	public void setState(LEDState state){
		ledState = state;
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
			  if (DriverStation.isDisabled()) {
			  //Fire goes here
			  }
			  else {
			  setColorRGB(0, 0, 0);
			  }
			  break;
			case CONE:
              setColorRGB(100, 100, 0);
			  break;
			case CUBE:
              setColorRGB(100, 0, 100);
			  break;
		}

}
}
