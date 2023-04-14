// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  public enum VisionState {
    DRIVE,
    TRACKING
  }

  public VisionState visionState = VisionState.DRIVE;

  public PhotonCamera visionCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  public PhotonPipelineResult result;
  public PhotonTrackedTarget target;

  public static final AprilTagFieldLayout aprilTagFieldLayout = createFieldLayout();

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.AVERAGE_BEST_TARGETS,
      visionCamera,
      Constants.Vision.cameraToRobot);

  public Vision() {
    visionCamera.setDriverMode(true);
  }

  public VisionState getState() {
    return visionState;
  }

  public void setState(VisionState state) {
    visionState = state;
  }

  private static AprilTagFieldLayout createFieldLayout() {
    try {
      return new AprilTagFieldLayout(Filesystem
          .getDeployDirectory()
          .toPath()
          .resolve("April_Tag_Layout.json"));
    } catch (IOException e) {
      throw new Error(e);
    }
  }

  /**
   * Set variable "target" to the latest and best fitting target tracked by the
   * camera.
   */
  public void recieveTarget() {
    if (result != null) {
      if (result.hasTargets()) {
        target = result.getBestTarget();
      }
    }
  }

  /** @return The X (forward/back) distance from the target */
  public double getX() {
    if (visionState.equals(VisionState.TRACKING)) {
      if (result.hasTargets()) {
        return target.getBestCameraToTarget().getX();
      }
    }
    return 0;
  }

  /** @return The Y (left/right) distance from the target */
  public double getY() {
    if (visionState.equals(VisionState.TRACKING)) {
      if (result.hasTargets()) {
        return target.getBestCameraToTarget().getY();
      }
    }
    return 0;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
        // The field layout failed to load, so we cannot estimate poses.
        return Optional.empty(); 
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
}

  public void getCameraResult() {
    result = visionCamera.getLatestResult();
  }

  public void setDriverMode(boolean driverMode) {
     visionCamera.setDriverMode(driverMode);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Vision State", String.valueOf(getState()));

    switch (visionState) {
      case DRIVE:
        setDriverMode(true);
        break;
      case TRACKING:
        setDriverMode(false);
        getCameraResult();
        recieveTarget();
        if (result.hasTargets()) {
          // System.out.println(PhotonUtils.estimateFieldToRobotAprilTag(
          // target.getBestCameraToTarget(),
          // aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
          // Constants.Vision.cameraToRobot));

        }
    }
  }
}
