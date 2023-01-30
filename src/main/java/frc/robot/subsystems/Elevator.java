// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

  public enum ElevatorState{ 
    IDLE,
    MANUAL,
    CLOSED_LOOP
  }

  public ElevatorState elevatorState = ElevatorState.IDLE;

  public CANSparkMax elevatorA = new CANSparkMax(Constants.Ports.ELEVATOR_A, MotorType.kBrushless);
  public CANSparkMax elevatorB = new CANSparkMax(Constants.Ports.ELEVATOR_B, MotorType.kBrushless);
  // Pivot on the elevator rack
  public CANSparkMax shoulder = new CANSparkMax(Constants.Ports.ARM, MotorType.kBrushless);

  public RelativeEncoder encoderA = elevatorA.getEncoder();
  public RelativeEncoder encoderShoulder = shoulder.getEncoder();

  public SparkMaxPIDController controllerA = elevatorA.getPIDController();
  public SparkMaxPIDController controllerShoulder = shoulder.getPIDController();

  public ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(Constants.Elevator.ELEVATOR_KS, 
                                                                  Constants.Elevator.ELEVATOR_KG, 
                                                                  Constants.Elevator.ELEVATOR_KV);

  public ArmFeedforward shoulderFeedForward = new ArmFeedforward(Constants.Elevator.SHOULDER_KS, 
                                                                Constants.Elevator.SHOULDER_KG, 
                                                                Constants.Elevator.SHOULDER_KV);

  // rotations
  public double setpointElevator = 0;
  public double setpointShoulder = 0;

  public Elevator() {
   elevatorB.follow(elevatorA, false);

   elevatorA.setIdleMode(IdleMode.kBrake);
   elevatorB.setIdleMode(IdleMode.kBrake);
   shoulder.setIdleMode(IdleMode.kBrake);

   elevatorA.setInverted(false);
  //  elevatorB.setInverted(false);
   shoulder.setInverted(false);

   elevatorA.setSoftLimit(SoftLimitDirection.kForward, 0);
   elevatorA.setSoftLimit(SoftLimitDirection.kReverse, 0);
   elevatorB.setSoftLimit(SoftLimitDirection.kForward, 0);
   elevatorB.setSoftLimit(SoftLimitDirection.kReverse, 0);
   shoulder.setSoftLimit(SoftLimitDirection.kForward, 0);
   shoulder.setSoftLimit(SoftLimitDirection.kReverse, 0);

   controllerA.setP(Constants.Elevator.ELEVATOR_KP);
   controllerA.setI(Constants.Elevator.ELEVATOR_KI);
   controllerA.setD(Constants.Elevator.ELEVATOR_KD);

   controllerShoulder.setP(Constants.Elevator.SHOULDER_KP);
   controllerShoulder.setI(Constants.Elevator.SHOULDER_KI);
   controllerShoulder.setD(Constants.Elevator.SHOULDER_KD);

   elevatorA.burnFlash();
   elevatorB.burnFlash();
   shoulder.burnFlash();
  }

  public void setState(ElevatorState state) {
    elevatorState = state;
  }

  public ElevatorState getState() {
    return elevatorState;
  }

  public void setElevatorOpenLoop(double percent) {
    elevatorA.set(percent);
  }

  // rotations
  public void setElevatorClosedLoop(double position) {
    controllerA.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void setShoulderOpenLoop(double percent) {
    shoulder.set(percent);
  }
  
  // rotations
  public void setShoulderClosedLoop(double position) {
    controllerA.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    switch(elevatorState) {
      case IDLE:
        setElevatorClosedLoop(0);
        setShoulderClosedLoop(0);
        break;
      case MANUAL:
        setElevatorOpenLoop(RobotContainer.operatorPad.getLeftY());
        break;
      case CLOSED_LOOP:
        setElevatorClosedLoop(setpointElevator);
        setShoulderClosedLoop(setpointShoulder);
        break;
    }
  }
}
