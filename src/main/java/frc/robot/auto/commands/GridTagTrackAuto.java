package frc.robot.auto.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionState;

public class GridTagTrackAuto extends CommandBase {

    private double xAxis = 0; 
    private double yAxis = 0;
    private double rotation = 0;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    private double ySetPoint = 0.40;

    private Drive drive;
    private Vision vision;

    private PIDController yController = new PIDController(0.3, 0.0, 0);
    private PIDController xController = new PIDController(0.6, 0.1, 0.0);

    public GridTagTrackAuto(Drive drive, Vision vision, boolean fieldRelative, boolean openLoop, double ySetPoint) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive, vision);
        
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.ySetPoint = ySetPoint;

        drive.resetOdometry(new Pose2d(new Translation2d(vision.getX(), vision.getY()), new Rotation2d()));
                                                  
        // rotController.reset();
        // rotController.setIntegratorRange(-0.2, 0.2);
        // yController.reset();
        // yController.setIntegratorRange(-0.2, 0.2);
        xController.reset();
        xController.setIntegratorRange(-0.2, 0.2);
    }

    @Override
    public void execute() {
        if (vision.getState().equals(VisionState.TRACKING)) {
        // rotation = rotController.calculate(Math.abs(drive.getAngle()), rotSetPoint);
        // rotation += yAxis;
        // if (drive.getAngle() < 0) {
        //     rotation *= -1;
        // }
        // rotation = MathUtil.clamp(rotation, -3.2, 3.2);
        // if (Math.abs(rotation) < 0.1) {
        //     rotation = 0;
        // }

        if (vision.result.hasTargets()) {
            yAxis = xController.calculate(drive.getPose().getY(), ySetPoint);
            yAxis = MathUtil.clamp(yAxis, -.2, .2);
            System.out.println(yAxis);
            if (Math.abs(yAxis) < 0.04) {
                yAxis = 0;
            }
            if (Math.abs(ySetPoint - drive.getPose().getY()) < 0.02) {
                yAxis = 0;
            }
        }
        
        // (forward/back, left/right) the controller axis is rotated from the Translation 2d axis
        translation = new Translation2d(xAxis, -1 * yAxis).times(Constants.Swerve.MAX_SPEED);
        drive.drive(translation, rotation, fieldRelative, openLoop);
        }
    }

    @Override
    public boolean isFinished() {
        if(vision.getState().equals(VisionState.DRIVE) || Math.abs(drive.getPose().getY() - ySetPoint) < 0.02) {
            return true;
        }
        return false;
    }
}    

