// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class BuddyClimb extends SubsystemBase {
// 	/** Creates a new BuddyClimb. */
// 	Solenoid liftA = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.BUDDYCLIMB_LIFT_A);
// 	Solenoid liftB = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.BUDDYCLIMB_LIFT_B);
// 	Solenoid deployA = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.BUDDYCLIMB_DEPLOY_A);
// 	Solenoid deployB = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.BUDDYCLIMB_DEPLOY_B);

// 	TalonFX winchA = new TalonFX(Constants.Ports.BUDDYCLIMB_WINCH_A);
// 	TalonFX winchB = new TalonFX(Constants.Ports.BUDDYCLIMB_WINCH_B);

// 	public BuddyClimb() {
// 		winchA.setNeutralMode(NeutralMode.Brake);
// 		winchB.setNeutralMode(NeutralMode.Brake);
// 	}

// 	public void setLift(boolean isLifted) {
// 		liftA.set(isLifted);
// 		liftB.set(isLifted);
// 	}

// 	public void setDeploy(boolean isDeployed) {
// 		deployA.set(isDeployed);
// 		deployB.set(isDeployed);
// 	}

// 	public void setWinch(double percent) {
// 		winchA.set(ControlMode.PercentOutput, percent);
// 		winchB.set(ControlMode.PercentOutput, percent);
// 	}

// 	@Override
// 	public void periodic() {
//		// This method will be called once per scheduler run
// 	}
// }
