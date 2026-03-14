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
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that continuously rotates the robot to face a specific angle using PID control 
 * while allowing translation.
 */
public class ContinuousRotateToAngle extends Command {
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
  public ContinuousRotateToAngle(CommandSwerveDrivetrain swerve, DoubleSupplier translationX, DoubleSupplier translationY) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004); // set PID directions
    this.translationX = translationX;
    this.translationY = translationY;
    pid.enableContinuousInput(-180, 180);

    SmartDashboard.putNumber("DRI - Rotate PID-P", pid.getP());
    SmartDashboard.putNumber("DRI - Rotate PID-I", pid.getI());
    SmartDashboard.putNumber("DRI - Rotate PID-D", pid.getD());
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
    Translation2d speakerPos = ShootingHelpers.getTargetPos();
    /*if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speakerPos = new Translation2d(0.02, 5.5826);
    } else {
      speakerPos = new Translation2d(16.4646, 5.5826);
    }*/

    degreeError = speakerPos.minus(Globals.EagleEye.position.getTranslation()).getAngle().getDegrees()
        - Globals.EagleEye.position.getRotation().getDegrees();
    degreeError = -degreeError;
    SmartDashboard.putNumber("SHO - Distance", speakerPos.minus(Globals.EagleEye.position.getTranslation()).getNorm());
    targetDegree = Globals.EagleEye.position.getRotation().getDegrees() - degreeError;

    if (targetDegree > 180) {
      targetDegree -= 360;
    } else if (targetDegree < -180) {
      targetDegree += 360;
    }

    SmartDashboard.putNumber("HOO - Target Angle", targetDegree);

    double P = SmartDashboard.getNumber("DRI - Rotate PID-P", 1.0 / 150.0);
    double I = SmartDashboard.getNumber("DRI - Rotate PID-I", 0.0);
    double D = SmartDashboard.getNumber("DRI - Rotate PID-D", 0);

    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    SmartDashboard.putNumber("DRI - Rotate Error", pid.getPositionError());
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
      swerve.drive(Math.pow(xIn, 3) * OperatorConstants.MAX_SPEED,
      Math.pow(yIn, 3) * OperatorConstants.MAX_SPEED, 
      0);
      // deadzone
    } else {
      swerve.drive(Math.pow(xIn, 3) * OperatorConstants.MAX_SPEED,
      Math.pow(yIn, 3) * OperatorConstants.MAX_SPEED, 
      direction);
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