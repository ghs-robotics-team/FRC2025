package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (89.8) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = 5; // 0.2/2.0 Min/Max

  public static final class SetPointConstants { //Dummy Values until measured
    public static final double ARM_LEFT_LOW = 3984.258;
    public static final double ARM_LEFT_MIDDLE = 3520.898;
    public static final double ARM_LEFT_HIGH = 3978.633;
    public static final double ARM_LEFT_TROUGH = 2330.332;

    public static final double ARM_RIGHT_LOW = -3984.258;
    public static final double ARM_RIGHT_MIDDLE = -3520.898;
    public static final double ARM_RIGHT_HIGH = -3978.633;
    public static final double ARM_RIGHT_TROUGH = -2330.332;

    public static final double ELEVATOR_HIGH = -33.066;
    public static final double ELEVATOR_MIDDLE = -17.256;
    public static final double ELEVATOR_LOW = -8.302;
    public static final double ELEVATOR_TROUGH = -2.158;
    public static final double ELEVATOR_INTAKE = -4.801;

  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(2.0, 0, 0.00);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT = 6;

    // Makes data taking easier
    public static final boolean SHOOTING_DATA_COLLECTION_MODE = false;

    // XBOX mode
    public static final boolean XBOX_DRIVE = false;

    public static final boolean MATT_MODE = true;
  }

  public static class EagleEyeConstants {
    public static final double MAX_VISION_SPEED = 2; // m/s
    public static final boolean IN_PATH_END = false;
  }

  public static class MaximumTemps {
    public static final double MaxNeoTemp = 90;
    public static final double MaxFalconTemp = 90;
  }
}