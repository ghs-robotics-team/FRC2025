// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;

/** Add your docs here. */
public final class Constants {
  //public static final double ROBOT_MASS = (89.8) * 0.453592; // 32lbs * kg per pound
  //public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = 2; // m/s

  public static class OperatorConstants 
  {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.05;
    public static final double LEFT_Y_DEADBAND  = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;

    public static final boolean XBOX_DRIVE = true;
  }
  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0); //new PIDConstants(5.0, 0.0, 0.0)
    public static final PIDConstants ANGLE_PID   = new PIDConstants(2.0, 0, 0.00); //new PIDConstants(5.0, 0.0, 0.0)
  }
  public static class MaximumTemps
  {
    public static final double MaxNeoTemp = 90;
    public static final double MaxFalconTemp = 90;
  }
}

