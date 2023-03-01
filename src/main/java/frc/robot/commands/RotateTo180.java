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

    private PIDController rotController = new PIDController(0.03, 0.5, 0.00);


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
      }

    @Override
    public void execute() {
        double xAxis = -controller.interpolatedLeftYAxis() * 0.7;
        double yAxis = -controller.interpolatedLeftXAxis() * 0.7;
        rotation = rotController.calculate(Math.abs(drive.getAngle()), rotSetPoint);
        rotation += yAxis;
        if (drive.getAngle() < 0) {
            rotation *= -1;
        }
        rotation = MathUtil.clamp(rotation, -3.2, 3.2);
        if (Math.abs(rotation) < 0.1) {
            rotation = 0;
        }
        
        translation = new Translation2d(yAxis, xAxis);
        drive.drive(translation, rotation, fieldRelative, openLoop);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}    

