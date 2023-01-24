package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class TeleopAprilTags extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
  
    private Drive drive;
    private Vision vision;
    private InterpolatedPS4Gamepad controller;

    /**
     * Driver control
     */
    public TeleopAprilTags(Drive drive, Vision vision, InterpolatedPS4Gamepad controller, boolean fieldRelative, boolean openLoop) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive, vision);

        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        if (RobotState.isTeleop()) {
            double yAxis = -controller.interpolatedLeftYAxis();
            double xAxis = -controller.interpolatedLeftXAxis();
            if (vision.result.hasTargets()) {
                System.out.println(vision.result.hasTargets());
                double error = vision.result.getBestTarget().getYaw();
                rotation = error * Constants.Swerve.KP_APRIL_TAGS * -1;
            }
            else {
                rotation = 0;
            }

            translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.MAX_SPEED);
            drive.drive(translation, rotation, fieldRelative, openLoop);
        }
    }
}
