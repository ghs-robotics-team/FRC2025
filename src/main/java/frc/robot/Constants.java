// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (89.8) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = 2;
  public static final class UnitConversionConstants{
    public static final double INDEXER_TICKS_PER_DEGREES = 0.4; //To be calculated later
  }

  public static final class SetPointConstants{
    public static final double ARM_AMP = -58.873;
    public static final double ARM_HUMAN_INTAKE = -50;
    public static final double ARM_INTAKE = 0;
    public static final double ARM_SPEAKER = -26.00; //-31.00;
    public static final double ARM_MAX = 0;
    public static final double ARM_MIN = -67.24;
    public static final double ARM_FAR = -42;
  }

  public static final class AutonConstants
  {

   public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0);
   public static final PIDConstants ANGLE_PID   = new PIDConstants(2.0, 0, 0.00);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.05;
    public static final double LEFT_Y_DEADBAND  = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;

    // Controller configuration
    public static final boolean SINGLE_XBOX = false;

    // Makes data taking easier
    public static final boolean SHOOTING_DATA_COLLECTION_MODE = false;

    // XBOX mode
    public static final boolean XBOX_DRIVE = false;
  }

  public static class EagleEyeConstants{
    public static final boolean USE_OLD_EAGLE_EYE = false;
    public static final boolean USE_MEGATAG_V1 = false;
    public static final double MAX_VISION_SPEED = 2;//m/s
  }
  
  public static class MaximumTemps
  {
    public static final double MaxNeoTemp = 90;
    public static final double MaxFalconTemp = 90;
  }
}