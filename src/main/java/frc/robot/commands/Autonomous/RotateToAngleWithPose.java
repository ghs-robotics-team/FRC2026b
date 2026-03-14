// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that rotates the robot to face a specific angle using PID control.
 */
public class RotateToAngleWithPose extends Command {
  private CommandSwerveDrivetrain swerve;
  private double degreeError;
  private PIDController pid;
  private double directionFactor;
  private double targetDegree;
  private Translation2d speakerPos;

  /**
   * Sets SwerveSubsystem and initializes PIDController.
   * 
   * @param swerve
   */
  public RotateToAngleWithPose(CommandSwerveDrivetrain swerve, Pose2d pose) {
    this.swerve = swerve;
    this.speakerPos = pose.getTranslation();
    this.pid = new PIDController(0.08, 0, 0.004);
    pid.enableContinuousInput(-180, 180);
    addRequirements(swerve);

    SmartDashboard.putNumber("DRI - Rotate PID-P", pid.getP());
    SmartDashboard.putNumber("DRI - Rotate PID-I", pid.getI());
    SmartDashboard.putNumber("DRI - Rotate PID-D", pid.getD());
  }

  /**
   * Stops the robot, gets seen targets from Limelight, gets angle error to
   * target,
   * and calculates target angle to rotate to while updating SmartDashboard.
   */
  @Override
  public void initialize() {
    swerve.drive(0 , 0 , 0);
    degreeError = 0;
     degreeError =
     speakerPos.minus(swerve.getState().Pose.getTranslation()).getAngle().getDegrees()
     - swerve.getState().Pose.getRotation().getDegrees();
    degreeError = -degreeError;

    SmartDashboard.putNumber("distance",
    speakerPos.minus(swerve.getState().Pose.getTranslation()).getNorm());
    targetDegree = swerve.getState().Pose.getRotation().getDegrees() - degreeError;

    if (targetDegree > 180) {
      targetDegree -= 360;
    } else if (targetDegree < -180) {
      targetDegree += 360;
    }

    SmartDashboard.putNumber("targetDegree", targetDegree);
  }

  /**
   * Continously calculates the angle error and moves the robot to face the target
   * angle.
   */
  @Override
  public void execute() {
    double P = SmartDashboard.getNumber("DRI - Rotate PID-P", 1.0 / 150.0);
    double I = SmartDashboard.getNumber("DRI - Rotate PID-I", 0.0);
    double D = SmartDashboard.getNumber("DRI - Rotate PID-D", 0);

    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    // Calculates power using PID to move the motors to the target angle.
    SmartDashboard.putNumber("DRI - Rotate Error", pid.getPositionError());
    directionFactor = pid.calculate(swerve.getState().Pose.getRotation().getDegrees(), targetDegree);

    if (directionFactor < 0.06 && directionFactor > -0.06) {
      directionFactor = Math.copySign(0.06, directionFactor);
    }

    // Fits direction into -4 to 4 range for swerve drive.
    directionFactor = MathUtil.clamp(directionFactor, -4, 4);
    SmartDashboard.putNumber("DRI - Rotate Output", directionFactor);
    if (pid.getPositionError() > -0.25 && pid.getPositionError() < 0.25) {
      // Dead Zone
      swerve.drive(0 , 0, 0);
    } else {
      swerve.drive(0, 0, directionFactor);
    }
  }

  /**
   * Stops the robot when the command ends.
   */
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0 , 0, 0);
  }

  /**
   * Never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}