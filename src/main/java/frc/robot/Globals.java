package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Global variables and settings for the robot.
 */
public final class Globals {

  /**
   * EagleEye vision and pose estimation data.
   */
  public static class EagleEye {
    public static Pose2d position = new Pose2d();
    public static double xVel = 0;
    public static double yVel = 0;
    public static double rotVel = 0;
    public static double rawGyroYaw = 0;
  }

  /**
   * Last vision measurement data for EagleEye.
   */
  public static class LastVisionMeasurement {
    public static Pose2d positionA = new Pose2d();
    public static Pose2d positionB = new Pose2d();
    public static double timeStampA = 0;
    public static double timeStampB = 0;
    public static boolean notRead = false;
    public static double confidenceA = 0;
    public static double confidenceB = 0;
  }

  /**
   * Last temperature measurements for shooter motors.
   */
  public static class LastTempMeasurement {
    public static double topShooterTemp = 0;
    public static double bottomShooterTemp = 0;
  }

  /**
   * Target positions for arm and elevator.
   */
  public static class targetPos {
    public static double intakeDeployTarget = 0;
  }

  /**
   * Shooting data collection settings.
   */
  public static class shootingDataCollectionSettings {
    public static boolean recording = false;
    public static boolean lastButtonState = false;
    public static Timer timer = new Timer();
    public static Pose2d startPose = new Pose2d();
    public static Pose2d endPose = new Pose2d();
  }

  // Miscellaneous global variables
  public static boolean inPath = false;
  public static XboxController buttonsXbox;
  public static double inversion = 1;
}