package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.EagleEyeConstants;

public class EagleEye extends SubsystemBase {
  /** Creates a new EagleEye. */
  public EagleEye() {
  }

  // private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new
  // Translation2d(0, 0), new Translation2d(16.541, 8.211));

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotBase.isSimulation())
      return;

    // If we don't update confidence then we don't send the measurement
    double confidence = 0;

    // Gets robot orientation from Gyro
    LimelightHelpers.SetRobotOrientation("limelight-cama", Globals.EagleEye.position.getRotation().getDegrees(), 0, 0,
        0, 0, 0);

    // Gets predicted location based on Tag
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
        .getBotPoseEstimate_wpiBlue("limelight-cama");

    SmartDashboard.putNumber("EE NumTags", limelightMeasurement.tagCount);
    SmartDashboard.putNumber("EE Avg Tag Dist", limelightMeasurement.avgTagDist);
    SmartDashboard.putNumber("EE Rotation Vel", Globals.EagleEye.rotVel);
    SmartDashboard.putNumber("EE Total Vel", Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

    // No tag found so check no further or pose not within field boundary
    if (limelightMeasurement.tagCount >= 1/* && fieldBoundary.isPoseWithinArea(limelightMeasurement.pose) */) {
      // Excluding different measurements that are absolute showstoppers even with
      // full trust
      if (limelightMeasurement.avgTagDist < Units.feetToMeters(15) && Globals.EagleEye.rotVel < Math.PI
          && Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel) < EagleEyeConstants.MAX_VISION_SPEED) {
        // Reasons to blindly trust as much as odometry
        if (DriverStation.isDisabled() ||
            (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagDist < Units.feetToMeters(10))) {
          confidence = 0.2;
        } else {
          // High trust level anything less than this we shouldn't bother with
          double compareDistance = limelightMeasurement.pose.getTranslation()
              .getDistance(Globals.EagleEye.position.getTranslation());
          if (compareDistance < 0.5 ||
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
  }
}
