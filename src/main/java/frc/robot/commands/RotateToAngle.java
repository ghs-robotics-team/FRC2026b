// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
//import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command that rotates the robot to face a specific angle using PID control.
 */
public class RotateToAngle extends Command {
  //private SwerveSubsystem swerve;
  private double degreeError;
  private PIDController pid;
  private double directionFactor;
  private double targetDegree;

  /**
   * Sets SwerveSubsystem and initializes PIDController.
   * 
   * @param swerve
   */
  public RotateToAngle(/*SwerveSubsystem swerve*/) {
    //addRequirements(swerve);
    //this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004);
    pid.enableContinuousInput(-180, 180);

    SmartDashboard.putNumber("PID-P", pid.getP());
    SmartDashboard.putNumber("PID-I", pid.getI());
    SmartDashboard.putNumber("PID-D", pid.getD());
  }

  /**
   * Stops the robot, gets seen targets from Limelight, gets angle error to
   * target,
   * and calculates target angle to rotate to while updating SmartDashboard.
   */
  @Override
  public void initialize() {
    //swerve.drive(new Translation2d(0, 0), 0, true);

    // Get fiducial (AprilTag Sightings) targets from Limelight
    LimelightTarget_Fiducial[] target_Fiducials = LimelightHelpers
        .getLatestResults("limelight-april").targets_Fiducials;
    degreeError = 0;

    for (LimelightTarget_Fiducial target : target_Fiducials) {
      if (target.fiducialID == 4) {
        degreeError = target.tx;
      }
    }

    // If no target seen, calculate angle to speaker based on position
    if (degreeError == 0) {
      Translation2d speakerPos;
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        speakerPos = new Translation2d(0.02, 5.5826);
      } else {
        speakerPos = new Translation2d(16.4646, 5.5826);
      }
      /*degreeError = speakerPos.minus(swerve.getPose().getTranslation()).getAngle().getDegrees()
          - swerve.getPose().getRotation().getDegrees();
      degreeError = -degreeError;
      SmartDashboard.putNumber("distance", speakerPos.minus(swerve.getPose().getTranslation()).getNorm());*/
    }

    Translation2d speakerPos;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speakerPos = new Translation2d(0.02, 5.5826);
    } else {
      speakerPos = new Translation2d(16.4646, 5.5826);
    }

    //SmartDashboard.putNumber("distance", speakerPos.minus(swerve.getPose().getTranslation()).getNorm());
    //targetDegree = swerve.getPose().getRotation().getDegrees() - degreeError;

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
    double P = SmartDashboard.getNumber("PID-P", 1.0 / 150.0);
    double I = SmartDashboard.getNumber("PID-I", 0.0);
    double D = SmartDashboard.getNumber("PID-D", 0);

    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    // Calculates power using PID to move the motors to the target angle.
    SmartDashboard.putNumber("currentError", pid.getPositionError());
    //directionFactor = pid.calculate(swerve.getPose().getRotation().getDegrees(), targetDegree);

    if (directionFactor < 0.06 && directionFactor > -0.06) {
      directionFactor = Math.copySign(0.06, directionFactor);
    }

    // Fits direction into -4 to 4 range for swerve drive.
    directionFactor = MathUtil.clamp(directionFactor, -4, 4);
    SmartDashboard.putNumber("Old direction", directionFactor);
    if (pid.getPositionError() > -0.25 && pid.getPositionError() < 0.25) {
      // Dead Zone
      //swerve.drive(new Translation2d(0, 0), 0, true);
    } else {
      //swerve.drive(new Translation2d(0, 0), directionFactor, true);
    }
  }

  /**
   * Stops the robot when the command ends.
   */
  @Override
  public void end(boolean interrupted) {
    //swerve.drive(new Translation2d(0, 0), 0, true);
  }

  /**
   * Never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}