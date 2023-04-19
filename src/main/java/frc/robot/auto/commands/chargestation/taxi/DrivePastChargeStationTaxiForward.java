// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands.chargestation.taxi;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DrivePastChargeStationTaxiForward extends CommandBase {
Translation2d translation;
Drive drive;
double lowestRoll = 0;
double highestRoll = 0;
  // must have back facing charge station when initialized
  public DrivePastChargeStationTaxiForward(Drive drive) {
   addRequirements(drive);
   this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      translation = new Translation2d(0.4, 0).times(Constants.Swerve.MAX_SPEED);
    drive.drive(translation, 0, true, true);
    System.out.println(lowestRoll + "     AUTO    ");
    if(lowestRoll > drive.gyro.getRoll()){
      lowestRoll = drive.gyro.getRoll();
    }
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
   if(drive.gyro.getRoll() > -3 && drive.gyro.getRoll() < 3 && lowestRoll < -13 && highestRoll > 13) {
    new WaitCommand(.5);
    return true;
   }
   else {
    return false;
   } 
  }
}
