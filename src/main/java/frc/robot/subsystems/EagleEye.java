// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.EagleEyeConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class EagleEye extends SubsystemBase {
  /** Creates a new EagleEye. */
  public EagleEye() {
    SmartDashboard.putBoolean("Use old EagleEye", Constants.EagleEyeConstants.USE_OLD_EAGLE_EYE);
  }
  
  //private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0, 0), new Translation2d(16.541, 8.211));

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(RobotBase.isSimulation()) return;
    if(SmartDashboard.getBoolean("Use old EagleEye", EagleEyeConstants.USE_OLD_EAGLE_EYE)){
      int goodMeasurements = 0;
      LimelightTarget_Fiducial[] target_Fiducials = 
        LimelightHelpers.getLatestResults("limelight-april").targets_Fiducials;

      for(LimelightTarget_Fiducial target : target_Fiducials){
        if(target.getTargetPose_CameraSpace().getTranslation().getNorm() < 6){
          goodMeasurements++;
          if(goodMeasurements>=2){
            break;
          }
        }
      }

      SmartDashboard.putNumber("Eagleye", goodMeasurements);
      if(goodMeasurements >= 2 && Timer.getFPGATimestamp()-Globals.LastVisionMeasurement.timeStamp >1){
        Globals.LastVisionMeasurement.position = LimelightHelpers.getBotPose2d_wpiBlue("limelight-april");
        Globals.LastVisionMeasurement.timeStamp = 
          Timer.getFPGATimestamp() 
          - LimelightHelpers.getLatency_Capture("limelight-april")/1000.0 
          - LimelightHelpers.getLatency_Pipeline("limelight-april")/1000.0;
        Globals.LastVisionMeasurement.notRead = true;
      }
    }else if(Constants.EagleEyeConstants.USE_MEGATAG_V1){
      double confidence = 0; // If we don't update confidence then we don't send the measurement
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-april");
      SmartDashboard.putNumber("NumTags", limelightMeasurement.tagCount);
      SmartDashboard.putNumber("Avg Tag Dist", limelightMeasurement.avgTagDist);
      SmartDashboard.putNumber("Rot Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber("Total Vel", Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

      // No tag found so check no further or pose not within field boundary
      if(limelightMeasurement.tagCount >= 1/* && fieldBoundary.isPoseWithinArea(limelightMeasurement.pose)*/) {
        // Excluding different measurements that are absolute showstoppers even with full trust 
        if(limelightMeasurement.avgTagDist < Units.feetToMeters(15) && Globals.EagleEye.rotVel < Math.PI && Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel) < EagleEyeConstants.MAX_VISION_SPEED) {
          // Reasons to blindly trust as much as odometry
          if (DriverStation.isDisabled() || 
              (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagDist < Units.feetToMeters(10))) {
                confidence = 0.2;
          } else {
            // High trust level anything less than this we shouldn't bother with
            double compareDistance = limelightMeasurement.pose.getTranslation().getDistance(Globals.EagleEye.position.getTranslation());
            if( compareDistance < 0.5 ||
            (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagDist < Units.feetToMeters(20)) ||
            (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < Units.feetToMeters(10))) {
              double tagDistance = Units.metersToFeet(limelightMeasurement.avgTagDist);
              // Double the distance for solo tag
              if (limelightMeasurement.tagCount == 1) {
                tagDistance = tagDistance * 2;
              }
              // Add up to .2 confidence depending on how far away
              confidence = 0.7 + (tagDistance / 100);
            }
          }
        }
      }
      Globals.LastVisionMeasurement.position = limelightMeasurement.pose;
      Globals.LastVisionMeasurement.timeStamp = limelightMeasurement.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;
      Globals.LastVisionMeasurement.confidence = confidence;
    }else{
      double confidence = 0; // If we don't update confidence then we don't send the measurement
      LimelightHelpers.SetRobotOrientation("limelight-april", Globals.EagleEye.position.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-april");
      SmartDashboard.putNumber("NumTags", limelightMeasurement.tagCount);
      SmartDashboard.putNumber("Avg Tag Dist", limelightMeasurement.avgTagDist);
      SmartDashboard.putNumber("Rot Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber("Total Vel", Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

      // No tag found so check no further or pose not within field boundary
      if(limelightMeasurement.tagCount >= 1/* && fieldBoundary.isPoseWithinArea(limelightMeasurement.pose)*/) {
        // Excluding different measurements that are absolute showstoppers even with full trust 
        if(limelightMeasurement.avgTagDist < Units.feetToMeters(15) && Globals.EagleEye.rotVel < Math.PI && Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel) < EagleEyeConstants.MAX_VISION_SPEED) {
          // Reasons to blindly trust as much as odometry
          if (DriverStation.isDisabled() || 
              (/*limelightMeasurement.tagCount >= 2 && --removed bc multitag isn't needed anymore*/limelightMeasurement.avgTagDist < Units.feetToMeters(10))) {
                confidence = 0.1;
          } else {
            // High trust level anything less than this we shouldn't bother with
            double compareDistance = limelightMeasurement.pose.getTranslation().getDistance(Globals.EagleEye.position.getTranslation());
            if( compareDistance < 0.5 ||
            (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagDist < Units.feetToMeters(20)) ||
            (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < Units.feetToMeters(15))) {
              double tagDistance = Units.metersToFeet(limelightMeasurement.avgTagDist);
              // Add up to .2 confidence depending on how far away
              confidence = 0.5 + (tagDistance / 100);
            }
          }
        }
      }
      Globals.LastVisionMeasurement.position = limelightMeasurement.pose;
      Globals.LastVisionMeasurement.timeStamp = limelightMeasurement.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;
      Globals.LastVisionMeasurement.confidence = confidence;
    }
  }
}
