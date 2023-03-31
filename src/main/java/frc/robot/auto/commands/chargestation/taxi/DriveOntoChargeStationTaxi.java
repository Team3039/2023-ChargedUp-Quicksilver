// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands.chargestation.taxi;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DriveOntoChargeStationTaxi extends CommandBase {
Translation2d translation;
Drive drive;
double highestRoll = 0;
  // must have back facing charge station when initialized
  public DriveOntoChargeStationTaxi(Drive drive) {
   addRequirements(drive);
   this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (highestRoll < 10) {
      translation = new Translation2d(0.40, 0).times(Constants.Swerve.MAX_SPEED);
    }
    else if(drive.gyro.getRoll() < 11.5 && highestRoll > 12) {
      translation = new Translation2d(0.10, 0).times(Constants.Swerve.MAX_SPEED);
    }
    else {
      translation = new Translation2d(0.20, 0).times(Constants.Swerve.MAX_SPEED);
    }
    drive.drive(translation, 0, true, true);
    System.out.println(drive.gyro.getRoll() + "     AUTO    ");
    if(highestRoll < drive.gyro.getRoll()){
      highestRoll = drive.gyro.getRoll();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new Translation2d(), 0, true, true);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if(drive.gyro.getRoll() < 10.5 && highestRoll > 12) {
    System.out.println("Finished Docking");
    return true;
   }
   else {
    return false;
   } 
  }
}
