// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that rotates the robot to face the alliance wall using PID control.
 */
public class RotateToAllianceWall extends Command {
  private CommandSwerveDrivetrain swerve;
  private PIDController pid;
  private double directionFactor;
  private double targetDegree;

  /**
   * Sets SwerveSubsystem and initializes PIDController.
   * 
   * @param swerve
   */
  public RotateToAllianceWall(CommandSwerveDrivetrain swerve, Pose2d pose) {
    this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004);
    pid.enableContinuousInput(-180, 180);
    addRequirements(swerve);

    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      SmartDashboard.putNumber("DRI - Rotate Pose PID-P", pid.getP());
      SmartDashboard.putNumber("DRI - Rotate Pose PID-I", pid.getI());
      SmartDashboard.putNumber("DRI - Rotate Pose PID-D", pid.getD());
    }
  }

  /**
   * Stops the robot, sets target angle based on alliance color.
   */
  @Override
  public void initialize() {
    swerve.drive(0 , 0 , 0);

    if (Alliance.Blue.equals(DriverStation.getAlliance())) {
      targetDegree = 180;
    }
    else{
      targetDegree = 0;
    }
    
    SmartDashboard.putNumber("RAW - targetDegree", targetDegree);
  }

  /**
   * Continously calculates the angle error and moves the robot to face the target
   * angle.
   */
  @Override
  public void execute() {
    double P = SmartDashboard.getNumber("RAW - Rotate Pose PID-P", 1.0 / 150.0);
    double I = SmartDashboard.getNumber("RAW - Rotate Pose PID-I", 0.0);
    double D = SmartDashboard.getNumber("RAW - Rotate Pose PID-D", 0);

    // Clamp PID values to reasonable ranges
    P = Math.max(0.0, Math.min(0.5, P));
    I = Math.max(0.0, Math.min(0.01, I));
    D = Math.max(0.0, Math.min(0.05, D));

    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      SmartDashboard.putNumber("RAW - Rotate Pose PID-P", P);
      SmartDashboard.putNumber("RAW - Rotate Pose PID-I", I);
      SmartDashboard.putNumber("RAW - Rotate Pose PID-D", D);
    }

    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    // Calculates power using PID to move the motors to the target angle.
    SmartDashboard.putNumber("RAW - Rotate Error", pid.getPositionError());
    directionFactor = pid.calculate(swerve.getState().Pose.getRotation().getDegrees(), targetDegree);

    if (directionFactor < 0.06 && directionFactor > -0.06) {
      directionFactor = Math.copySign(0.06, directionFactor);
    }

    // Fits direction into -4 to 4 range for swerve drive.
    directionFactor = MathUtil.clamp(directionFactor, -4, 4);
    SmartDashboard.putNumber("RAW - Rotate Output", directionFactor);
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