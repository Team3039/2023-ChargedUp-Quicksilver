
package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.LockWheels;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.auto.commands.chargestation.normal.ChargeStationBalance;
import frc.robot.auto.commands.chargestation.normal.DriveOntoChargeStation;
import frc.robot.commands.ElevatorRoutines.ActuateLowToHighGrid;
import frc.robot.commands.ElevatorRoutines.ActuateToIdle;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeStationAuto extends SequentialCommandGroup {

  /** Creates a new ChargeStationAuto. */
  public ChargeStationAuto(Drive s_Swerve) {
    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
        new InstantCommand(() -> s_Swerve.setGyro(0)),
        new ActuateLowToHighGrid(),
        new SetClawReleaseMode(),
        new WaitCommand(0.5),
        new SetClawIdleMode(),
        new ActuateToIdle(),
        new DriveOntoChargeStation(s_Swerve),
        new ChargeStationBalance(s_Swerve),
        new LockWheels(s_Swerve),
        new InstantCommand(() -> s_Swerve.setGyro(180)));
  }
}