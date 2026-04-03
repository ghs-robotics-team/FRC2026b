// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

/**
 * Command to run the shooter at a specific RPM using a PID controller in the Shooter subsystem.
 */
public class ShootingRPMCommand extends Command {
  private Shooter shooter;
  private double rpm;

  /**
   * Creates a new ShootingOnlyCommand.
   * @param shooter The shooter subsystem to run.
   * @param power The power level to set the shooter motor to, typically between -1.0 and 1.0.
   */
  public ShootingRPMCommand(Shooter shooter, double rpm) {
    this.shooter = shooter;
    this.rpm = rpm;
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL) {
      SmartDashboard.putNumber("Shooting RPM Override", rpm);
    }
    addRequirements(shooter);
  }

  /**
   * Stop the shooter motors.
   */
  @Override
  public void initialize() {
    shooter.shootTargetSpeed(0);
  }

  /**
   * Runs the shooter at the specified power level.
   */
  @Override
  public void execute() {
    double rpmToUse = rpm;
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL) {
      double dashboardRpm = SmartDashboard.getNumber("Shooting RPM Override", 0.0);
      if (dashboardRpm != 0) {
        rpmToUse = dashboardRpm;
      }
    }
    shooter.shootTargetSpeed(rpmToUse);
  }

  /**
   * Stop the shooter motors.
   */
  @Override
  public void end(boolean interrupted) {
    shooter.shootTargetSpeed(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */
  public boolean isFinished() {
    return false;
  }
}
