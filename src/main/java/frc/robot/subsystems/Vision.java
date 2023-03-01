// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  public enum VisionState {
    DRIVE,
    TRACKING
  }

  public VisionState visionState = VisionState.DRIVE;

  public PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  public PhotonPipelineResult result;
  public PhotonTrackedTarget target;

  public Vision() {
    camera.setDriverMode(false);
  }

  public VisionState getState() {
    return visionState;
  }

  public void setState(VisionState state) {
    visionState = state;
  }

  /**
   * Set variable "target" to the latest and best fitting target tracked by the
   * camera.
   */
  public void recieveTarget() {
    if (result.hasTargets()) {
      target = result.getBestTarget();
    }
  }
  /** @return The X (forward/back) distance from the target */
  public double getX() {
    if (result.hasTargets()) {
      return target.getBestCameraToTarget().getX();
    }
    return 0;
  }

  /** @return The X (left/right) distance from the target */
  public double getY() {
    if (result.hasTargets()) {
      return target.getBestCameraToTarget().getY();
    }
    return 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Vision State", String.valueOf(getState()));
    
    switch(visionState) {
      case DRIVE:
      camera.setDriverMode(true);
      break;
      case TRACKING:
      camera.setDriverMode(false);
      result = camera.getLatestResult();
      recieveTarget();
    }
  }
}
