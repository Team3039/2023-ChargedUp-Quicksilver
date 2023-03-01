package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionState;

public class RotateTo180 extends CommandBase {

    private double rotation = 0;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    private double rotSetPoint = 180;
    private double xAxis = 0;
    private double yAxis = 0;

    private Drive drive;
    private InterpolatedPS4Gamepad controller;

    private PIDController rotController = new PIDController(0.02, 0.5, 0.00);

    // 1.27 m	-0.59 m	 178.24°	

    /**
     * Driver control
     */
    public RotateTo180(Drive drive, InterpolatedPS4Gamepad controller, boolean fieldRelative, boolean openLoop, double ySetPoint) {
        this.drive = drive;
        addRequirements(drive);
        
        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        rotController.reset();
        rotController.setIntegratorRange(-0.2, 0.2);
        // yController.reset();
        // yController.setIntegratorRange(-0.2, 0.2);
      }

    @Override
    public void execute() {
        double xAxis = -controller.interpolatedLeftYAxis();
        double yAxis = -controller.interpolatedLeftXAxis();
        // if (vision.getState().equals(VisionState.TRACKING)) {
        rotation = rotController.calculate(Math.abs(drive.getAngle()), rotSetPoint);
        rotation += yAxis;
        if (drive.getAngle() < 0) {
            rotation *= -1;
        }
        rotation = MathUtil.clamp(rotation, -3.2, 3.2);
        if (Math.abs(rotation) < 0.1) {
            rotation = 0;
        }

        // if (vision.result.hasTargets()) {
        //     yAxis = xController.calculate(vision.result.getBestTarget().getBestCameraToTarget().getY(), ySetPoint);
        //     yAxis = MathUtil.clamp(yAxis, -.2, .2);
        //     System.out.println(yAxis);
        //     if (Math.abs(yAxis) < 0.04) {
        //         yAxis = 0;
        //     }
        // }
        // else {
        //     yAxis = -.7 * controller.interpolatedLeftXAxis();
        // }
        
        // (forward/back, left/right) the controller axis is rotated from the Translation 2d axis
        // translation = new Translation2d(xAxis, yAxis).times(Constants.Swerve.MAX_SPEED);
        translation = new Translation2d(yAxis, xAxis);
        drive.drive(translation, rotation, fieldRelative, openLoop);
        // }
    }

    @Override
    public boolean isFinished() {
        // if(vision.getState().equals(VisionState.DRIVE)) {
        //     return true;
        // }
        // return false;
        return false;
    }
}    

