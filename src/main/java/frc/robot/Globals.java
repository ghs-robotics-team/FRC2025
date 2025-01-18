// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public final class Globals
{
  public static class EagleEye {
    public static Pose2d position = new Pose2d();
    public static double xVel = 0;
    public static double yVel = 0;
    public static double rotVel = 0;
  }
  public static class LastVisionMeasurement {
    public static Pose2d position = new Pose2d();
    public static double timeStamp = 0;
    public static boolean notRead = false;
    public static double confidence = 0;
  }
  public static class LastTempMeasurement {
    public static double topShooterTemp = 0;
    public static double bottomShooterTemp = 0;
  }
}
