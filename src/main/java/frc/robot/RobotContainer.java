// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ActuateToSetpoint;
import frc.robot.commands.ClawIntake;
import frc.robot.commands.ClawRelease;
import frc.robot.commands.GridTagTrack;
import frc.robot.commands.SetLEDS;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TrackingMode;
import frc.robot.commands.ElevatorRoutines.ActuateLowToHighGrid;
import frc.robot.commands.ElevatorRoutines.ActuateLowToMidGrid;
import frc.robot.commands.ElevatorRoutines.ActuateLowToSingleStation;
import frc.robot.commands.ElevatorRoutines.ActuateToIdle;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.BuddyClimb;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Vision vision = new Vision();
  public static final Drive drive = new Drive();
  public static final Claw claw = new Claw();
  public static final Elevator elevator = new Elevator();
  public static final Wrist wrist = new Wrist();
  public static final BuddyClimb buddyClimb = new BuddyClimb();
  public static final LEDs ledsLeft = new LEDs();

  public static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(1);
  public static final InterpolatedPS4Gamepad operatorPad = new InterpolatedPS4Gamepad(2);
  
  private final JoystickButton driverX = new JoystickButton(driverPad, PS4Controller.Button.kCross.value);
  private final JoystickButton driverSquare = new JoystickButton(driverPad, PS4Controller.Button.kSquare.value);
  private final JoystickButton driverTriangle = new JoystickButton(driverPad, PS4Controller.Button.kTriangle.value);
  private final JoystickButton driverCircle = new JoystickButton(driverPad, PS4Controller.Button.kCircle.value);

  // private final JoystickButton driverDPadUp = new JoystickButton(driverPad, );
  // private final JoystickButton driverDPadDown = new JoystickButton(driverPad, PS4Controller.Button.DPAD_DOWN);
  // private final JoystickButton driverDPadLeft = new JoystickButton(driverPad, PS4Controller.Button.DPAD_LEFT);
  // private final JoystickButton driverDPadRight = new JoystickButton(driverPad, PS4Controller.Button.DPAD_RIGHT);

  private final JoystickButton driverL1 = new JoystickButton(driverPad, PS4Controller.Button.kL1.value);
  private final JoystickButton driverR1 = new JoystickButton(driverPad, PS4Controller.Button.kR1.value);

  private final JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);
  private final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kR2.value);

  private final JoystickButton driverPadButton = new JoystickButton(driverPad, PS4Controller.Button.kTouchpad.value);
  private final JoystickButton driverStart = new JoystickButton(driverPad, PS4Controller.Button.kPS.value);

  private final JoystickButton driverShare = new JoystickButton(driverPad, PS4Controller.Button.kShare.value);
  private final JoystickButton driverOptions = new JoystickButton(driverPad, PS4Controller.Button.kOptions.value);

  /* Operator Buttons */
  private final JoystickButton operatorX = new JoystickButton(operatorPad, PS4Controller.Button.kCross.value);
  private final JoystickButton operatorSquare = new JoystickButton(operatorPad, PS4Controller.Button.kSquare.value);
  private final JoystickButton operatorTriangle = new JoystickButton(operatorPad, PS4Controller.Button.kTriangle.value);
  private final JoystickButton operatorCircle = new JoystickButton(operatorPad, PS4Controller.Button.kCircle.value);

  // private final JoystickButton operatorDPadUp = new JoystickButton(operatorPad, );
  // private final JoystickButton operatorDPadDown = new JoystickButton(operatorPad, PS4Controller.Button.DPAD_DOWN);
  // private final JoystickButton operatorDPadLeft = new JoystickButton(operatorPad, PS4Controller.Button.DPAD_LEFT);
  // private final JoystickButton operatorDPadRight = new JoystickButton(operatorPad, PS4Controller.Button.DPAD_RIGHT);

  private final JoystickButton operatorL1 = new JoystickButton(operatorPad, PS4Controller.Button.kL1.value);
  private final JoystickButton operatorR1 = new JoystickButton(operatorPad, PS4Controller.Button.kR1.value);

  private final JoystickButton operatorL2 = new JoystickButton(operatorPad, PS4Controller.Button.kL2.value);
  private final JoystickButton operatorR2 = new JoystickButton(operatorPad, PS4Controller.Button.kR2.value);
  private final JoystickButton operatorR3 = new JoystickButton(operatorPad, PS4Controller.Button.kR3.value);

  private final JoystickButton operatorPadButton = new JoystickButton(operatorPad, PS4Controller.Button.kTouchpad.value);
  private final JoystickButton operatorStart = new JoystickButton(operatorPad, PS4Controller.Button.kPS.value);

  private final JoystickButton operatorShare = new JoystickButton(operatorPad, PS4Controller.Button.kShare.value);
  private final JoystickButton operatorOptions = new JoystickButton(operatorPad, PS4Controller.Button.kOptions.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(
      new TeleopSwerve(drive, 
      driverPad, 
      true, 
      true)
    );
    driverOptions.onTrue(new InstantCommand(() -> drive.setGyro(0)));

    driverShare.toggleOnTrue(new TrackingMode());
    driverSquare.onTrue(new GridTagTrack(drive, vision, driverPad, true, true, -0.4));
    driverCircle.onTrue(new GridTagTrack(drive, vision, driverPad, true, true, 0.4));
    driverTriangle.onTrue(new GridTagTrack(drive, vision, driverPad, true, true, 0.0));

    driverL1.onTrue(new SetLEDS());

    // operatorSquare.whileTrue(new SetElevatorPercent(.20));
    // operatorTriangle.whileTrue(new SetElevatorPercent(-.10));
    // operatorR1.whileTrue(new SetWristPercent(0.1));
    // operatorR2.whileTrue(new SetWristPercent(-0.1));

    // operatorSquare.toggleOnTrue(new ActuateToSetpoint(75, 20));
    // operatorX.toggleOnTrue(new ActuateToSetpoint(45, 0));
    // operatorTriangle.toggleOnTrue(new ActuateToSetpoint(82, 60));

    operatorCircle.onTrue(new ActuateLowToSingleStation());
    operatorSquare.onTrue(new ActuateLowToMidGrid());
    operatorTriangle.onTrue(new ActuateLowToHighGrid());
    operatorX.onTrue(new ActuateToIdle());

    operatorL1.whileTrue(new ClawIntake());
    operatorL2.whileTrue(new ClawRelease());
  }

  public Command getAutonomousCommand() {
    return null;
  }

}