package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command that rotates the robot to face a specific target position on the field using PID control.
 */
public class FaceTargetCommand extends Command {
  private static final String kExecutingKey = "FT Executing";
  private static final String kRotatingKey = "FT Rotating";
  private static final String kTargetAngleKey = "FT TargetDeg";
  private static final String kErrorAngleKey = "FT ErrorDeg";
  private static final String kPKey = "FT P";
  private static final String kDKey = "FT D";
  private static final double kPDefault = 0.3;
  private static final double kDDefault = 0.01;
  private static final double kPMin = 0.0;
  private static final double kPMax = 5.0;
  private static final double kDMin = 0.0;
  private static final double kDMax = 0.5;
  private static final double kI = 0.0;
  private static final double kAngleToleranceDeg = 2.0;
  private static final double kMinAngularVelocityRadPerSec = 0.3;

  private static double cachedP = kPDefault;
  private static double cachedD = kDDefault;

  static {
    SmartDashboard.putBoolean(kExecutingKey, false);
    SmartDashboard.putBoolean(kRotatingKey, false);
    SmartDashboard.putNumber(kTargetAngleKey, 0.0);
    SmartDashboard.putNumber(kErrorAngleKey, 0.0);
    SmartDashboard.putNumber(kPKey, kPDefault);
    SmartDashboard.putNumber(kDKey, kDDefault);
  }

  //private final SwerveSubsystem swerve;
  private final Pose2d target;
  private final PIDController rotationController;
  //private final double maxAngularVelocity;

  /**
   * Sets SwerveSubsystem, target position, initializes PIDController, 
   * and adds swerve as a requirement.
   * @param swerve The SwerveSubsystem used to control the robot's movement.
   * @param target The target angle on the field to face.
   */
  public FaceTargetCommand(/*SwerveSubsystem swerve, */Pose2d target) {
    //this.swerve = swerve;
    this.target = target;
    this.rotationController = new PIDController(cachedP, kI, cachedD);
    this.rotationController.enableContinuousInput(-180.0, 180.0);
    this.rotationController.setTolerance(kAngleToleranceDeg);
    //this.maxAngularVelocity = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    //addRequirements(swerve);
  }

  /**
   * Initializes the command by updating SmartDashboard to indicate that the command is executing,
   * refreshing tunable PID values from SmartDashboard, setting the PIDController's P and D values,
   * and resetting the PIDController's internal state.
   */
  @Override
  public void initialize() {
    SmartDashboard.putBoolean(kExecutingKey, true);
    refreshTunables();
    rotationController.setP(cachedP);
    rotationController.setD(cachedD);
    rotationController.reset();
  }

  /**
   * Continuously calculates the angle error to the target and rotates the robot to face the target angle using PID control.
   * Also updates SmartDashboard with the current angle, target angle, error angle, and rotation output for debugging purposes.
   * If the robot is within the angle tolerance, it will stop rotating.
   * If the calculated rotation output is below the minimum angular velocity, 
   * it will be set to the minimum to ensure the robot continues rotating towards the target.
   */
  @Override
  public void execute() {
    Pose2d currentPose = new Pose2d();//swerve.getPose();
    Translation2d toTarget = target.getTranslation().minus(currentPose.getTranslation());

    if (toTarget.getNorm() < 1e-4) {
      //swerve.stop();
      return;
    }

    double targetAngle = toTarget.getAngle().getDegrees();
    double currentAngle = currentPose.getRotation().getDegrees();
    double errorAngle = MathUtil.inputModulus(targetAngle - currentAngle, -180.0, 180.0);

    SmartDashboard.putNumber("FT Cur Angle", currentAngle);
    SmartDashboard.putNumber(kTargetAngleKey, targetAngle);
    SmartDashboard.putNumber(kErrorAngleKey, errorAngle);

    double rotationOutputDegPerSec = rotationController.calculate(currentPose.getRotation().getDegrees(),
        targetAngle);
    SmartDashboard.putNumber("FT Rot Output", rotationOutputDegPerSec);

    double rotationOutput = Units.degreesToRadians(rotationOutputDegPerSec);
    SmartDashboard.putNumber("FT Rot Output RAD", rotationOutput);
    //rotationOutput = MathUtil.clamp(rotationOutput, -maxAngularVelocity, maxAngularVelocity);
    SmartDashboard.putNumber("FT Rot Output CLAMP", rotationOutput);

    boolean rotating = true;
    if (rotationController.atSetpoint()) {
      rotationOutput = 0.0;
      rotating = false;
    } else if (Math.abs(rotationOutput) < kMinAngularVelocityRadPerSec) {
      rotationOutput = Math.copySign(kMinAngularVelocityRadPerSec, rotationOutput);
    }

    SmartDashboard.putNumber("FT Rot Output POST MIN", rotationOutput);

    SmartDashboard.putBoolean(kRotatingKey, rotating);

    //swerve.drive(new Translation2d(), -rotationOutput, true);
  }

  /**
   * Stops the robot and updates SmartDashboard to indicate that the 
   * command is no longer executing or rotating when the command ends.
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    //swerve.stop();
    SmartDashboard.putBoolean(kExecutingKey, false);
    SmartDashboard.putBoolean(kRotatingKey, false);
  }

  /**
   * This command is designed to run indefinitely until interrupted, 
   * as it continuously adjusts the robot's orientation to face the target position.
   * @return False, indicating that the command should not end on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Refreshes the tunable PID values from SmartDashboard and updates the cached values, 
   * ensuring they are within the defined min and max limits.
   * This allows for real-time tuning of the PID controller while the command is running.
   */
  private static void refreshTunables() {
    double p = SmartDashboard.getNumber(kPKey, kPDefault);
    double d = SmartDashboard.getNumber(kDKey, kDDefault);
    cachedP = MathUtil.clamp(p, kPMin, kPMax);
    cachedD = MathUtil.clamp(d, kDMin, kDMax);
  }
}
