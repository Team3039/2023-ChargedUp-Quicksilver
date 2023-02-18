
package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

/**
 * This class generates auto commands.
 */
public class AutoCommands {
  private final Drive swerve;
  private final SendableChooser<Command> dropDown;

  
  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController thetaController = new PIDController(1, 0.0, 0.0);

  /**
   * Define all auto commands.
   */
  public AutoCommands(Drive swerve) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.swerve = swerve;

    dropDown = new SendableChooser<>();
    dropDown.addOption("Example", run(() -> {
      getCommand("Bottom 3 Piece YYP", true);
    }));

    SmartDashboard.putData("Auto Selection", dropDown);
 
  }

  public Command getSelectedCommand() {
    return dropDown.getSelected();
  }

  private Command getCommand(String pathName, boolean isFirstPath) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(
      pathName,
      3,
      5);

    return sequence(
      
      run(() -> {
        if (isFirstPath) {
          swerve.resetOdometry(traj.getInitialHolonomicPose());
        }
      }, swerve),

      new PPSwerveControllerCommand(
        traj,
        swerve::getPose,
        Constants.Swerve.SWERVE_KINEMATICS,
        xController,
        yController,
        thetaController,
        swerve::setModuleStates,
        swerve),

      new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false))
    );
  }
}