
package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.LockWheels;
import frc.robot.auto.commands.SetClawIdleMode;
import frc.robot.auto.commands.SetClawReleaseMode;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateLowToHighGridAuto;
import frc.robot.auto.commands.AutoElevatorRoutines.ActuateToIdleAuto;
import frc.robot.auto.commands.chargestation.normal.ChargeStationBalance;
import frc.robot.auto.commands.chargestation.normal.DriveOntoChargeStation;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeStationAuto extends SequentialCommandGroup {

  /** Creates a new ChargeStationAuto. */
  public ChargeStationAuto(Drive swerve) {
    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d())),
        new InstantCommand(() -> swerve.setGyro(0)),
        new ActuateLowToHighGridAuto(),
        new SetClawReleaseMode(),
        new WaitCommand(0.5),
        new SetClawIdleMode(),
        new ActuateToIdleAuto(),
        new DriveOntoChargeStation(swerve),
        new ChargeStationBalance(swerve),
        new LockWheels(swerve),
        new InstantCommand(() -> swerve.setGyro(180)));
  }
}