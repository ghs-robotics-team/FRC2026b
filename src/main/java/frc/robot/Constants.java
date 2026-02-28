package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

  /**
   * Unit conversion constants.
   */
  public static final class UnitConversionConstants {
    public static final double INDEXER_TICKS_PER_DEGREES = 0.4; // To be calculated later
  }

  /**
   * Preset setpoints upper mechanicals
   */
  public static final class SetPointConstants {
  }

  /**
   * Autonomous Drivetrain PID constants.
   */
  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(7.7, 0, 0.035); // D of 3?
    public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0, 0.00);
  }

  /**
   * Drivebase constants.
   */
  public static final class DrivebaseConstants {
    public static final String YAGSL_CONFIG_FOLDER = "swerve8701";
    public static final double WHEEL_LOCK_TIME = 10; // Hold time (seconds) on motor brakes when disabled
  }

  /**
   * Operator interface constants.
   */
  public static class OperatorConstants {

    // Makes testing safer and easier
    public static final boolean WORKSHOP_MODE = true;
    public static final double WORKSHOP_MAX_SPEED_MODIFIER = 0.2; // Percent as decimal
    public static final double WORKSHOP_MAX_ROTATIONS_PER_SECOND_MODIFIER = 0.5; // Percent as decimal
 
    public static final double MAX_SPEED_MODIFIER = WORKSHOP_MODE ? WORKSHOP_MAX_SPEED_MODIFIER : 1.0; // Percent as decimal
    public static final double MAX_ROTATIONS_PER_SECOND_MODIFIER = WORKSHOP_MODE ? WORKSHOP_MAX_ROTATIONS_PER_SECOND_MODIFIER : 0.75; // Percent as decimal

    // Joystick Deadband
    public static final double TRANSLATION_DEADBAND = 0.1; // Percent as decimal
    public static final double ROTATION_DEADBAND = 0.1; // Percent as decimal

    // Makes data taking easier
    public static final boolean SHOOTING_DATA_COLLECTION_MODE = false;

    // Dynamically Control Subsystem power
    public static final boolean  DYNAMIC_POWER_CONTROL = true;

    // XBOX mode
    public static final boolean XBOX_DRIVE = false;
  }

  /**
   * Eagle Eye vision system constants.
   */
  public static class EagleEyeConstants {
    public static final boolean EAGLEEYE_ENABLED = false;
    public static final boolean EAGLEEYE_DURING_AUTO = true;
    public static final double MAX_VISION_SPEED = 2.25; // Units are m/s. Usually 1.5-2.5 before it stops reading vision measurements
    public static final double MAX_TAG_DISTANCE_METERS = Units.feetToMeters(20);
    public static final double MAX_ROTATION_VELOCITY = Math.PI / 2; // Units are radians per second
    public static final boolean IN_PATH_END = false;
  }

  /**
   * Maximum temperature constants for motors.
   */
  public static class MaximumTemps {
    public static final double MAX_NEO_TEMP = 90;
    public static final double MAX_FALCON_TEMP = 90;
  }
}