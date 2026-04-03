// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.ShootingHelpers;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that continuously rotates the robot to face a specific angle using PID control 
 * while allowing translation.
 */
public class ContinuousRotateToAllianceWall extends Command {
  CommandSwerveDrivetrain swerve;
  double degreeError;
  PIDController pid;
  double direction;
  double targetDegree;
  DoubleSupplier translationX;
  DoubleSupplier translationY;
  public boolean found;

  /**
   * Sets SwerveSubsystem and initializes PIDController.
   * @param swerve The drivetrain subsystem to control. 
   * @param translationX The supplier for the x-component of translation.
   * @param translationY The supplier for the y-component of translation.
   */
  public ContinuousRotateToAllianceWall(CommandSwerveDrivetrain swerve, DoubleSupplier translationX, DoubleSupplier translationY) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004); // set PID directions
    this.translationX = translationX;
    this.translationY = translationY;
    pid.enableContinuousInput(-180, 180);

    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      SmartDashboard.putNumber("DRI - Rotate Continuous PID-P", 0.08);
      SmartDashboard.putNumber("DRI - Rotate Continuous PID-I", 0.0);
      SmartDashboard.putNumber("DRI - Rotate Continuous PID-D", 0.004);
    }
  }

  /**
   * Stops the robot at beginning of command.
   */
  @Override
  public void initialize() {
    swerve.drive(0, 0, 0);
  }

  /**
   * Continuously gets seen targets from Limelight, gets angle error to target,
   * and calculates target angle to rotate to while updating SmartDashboard.
   * Also allows translation control while rotating to target angle.
   */
  @Override
  public void execute() {
    if (Alliance.Blue.equals(DriverStation.getAlliance())) {
      targetDegree = 180;
    }
    else{
      targetDegree = 0;
    }

    SmartDashboard.putNumber("CRAW - Target Angle", targetDegree);

    double P = SmartDashboard.getNumber("CRAW - Rotate Continuous PID-P", 0.08);
    double I = SmartDashboard.getNumber("CRAW - Rotate Continuous PID-I", 0.0);
    double D = SmartDashboard.getNumber("CRAW - Rotate Continuous PID-D", 0.004);

    // Clamp PID values to safe ranges
    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      P = Math.max(0.0, Math.min(0.5, P));
      I = Math.max(0.0, Math.min(0.01, I));
      D = Math.max(0.0, Math.min(0.05, D));
      SmartDashboard.putNumber("CRAW - Rotate Continuous PID-P", P);
      SmartDashboard.putNumber("CRAW - Rotate Continuous PID-I", I);
      SmartDashboard.putNumber("CRAW - Rotate Continuous PID-D", D);
    }

    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    SmartDashboard.putNumber("CRAW - Rotate Error", pid.getPositionError());
    direction = pid.calculate(swerve.getState().Pose.getRotation().getDegrees(), targetDegree);

    if (direction < 0.06 && direction > -0.06) {
      direction = Math.copySign(0.06, direction);

    }

    direction = MathUtil.clamp(direction, -4, 4);
    SmartDashboard.putNumber("DRI - Rotate Output", direction);
    double xIn = 0;
    double yIn = 0;

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      xIn = -translationX.getAsDouble();
      yIn = -translationY.getAsDouble();
    } else {
      xIn = translationX.getAsDouble();
      yIn = translationY.getAsDouble();
    }
   
    if (pid.getPositionError() > -0.25 && pid.getPositionError() < 0.25) {
      // Dead Zone
      swerve.drive(xIn , yIn, 0);
    } else {
      swerve.drive(xIn, yIn, direction);
    }
  }

  /**
   * Stops the robot when the command ends.
    * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0 , 0);
  }

  /**
   * This command never finishes on its own and must be interrupted or canceled to end.
    * @return false, indicating the command should not end on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}