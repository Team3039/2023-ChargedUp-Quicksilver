// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  public PhotonCamera camera = new PhotonCamera("photonvision");
  public PhotonPipelineResult result;
  public PhotonTrackedTarget target;
  
  public double TargetX;

  public Vision() {
    camera.setDriverMode(false);
  }

  /**  Set variable "target" to the latest and best fitting target tracked by the camera.   */
  public void recieveTarget() {
    System.out.println("oh my [redacted] god");
    result = camera.getLatestResult();
    System.out.println(result.hasTargets());
    if (result.hasTargets()) {
      target = result.getBestTarget();
      System.out.println(target.getYaw());
    }
  }

  @Override
  public void periodic() {
   recieveTarget();

  }
}
