// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimEye extends SubsystemBase {
  /** Creates a new SimEye. */
  VisionSystemSim visionSim;
  public SimEye() {
    visionSim = new VisionSystemSim("main");

    TargetModel targetModel = TargetModel.kAprilTag36h11;

    Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
    VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
    visionSim.addVisionTargets(visionTarget);

    //AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    //visionSim.addAprilTags(tagLayout);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
