package frc.robot.auto.commands.chargestation.normal;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ChargeStationBalance extends CommandBase {
  Drive drive;
  Translation2d translation;
  Timer timer = new Timer();
  
  /** Creates a new ChargeStationBalance. */
  public ChargeStationBalance(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translation = new Translation2d(drive.getRoll() * .0089, 0).times(Constants.Swerve.MAX_SPEED);
    drive.drive(translation, 0, true, true);
    if (Math.abs(drive.getRoll()) < 5) {
      timer.start();
    }
    else {
      timer.stop();
      timer.reset();
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
    if (Math.abs(drive.getRoll()) < 4 && timer.advanceIfElapsed(1)) {
    System.out.println("Finished Balancing");
    return true;
    }
    else {
    return false;
    }
  }

}
