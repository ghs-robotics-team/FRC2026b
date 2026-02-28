// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Runs the shooter motor only.
 */
public class ShootingOnlyCommand extends Command {
  Shooter shooter;
  double power;

  /**
   * Creates a new ShootingOnlyCommand.
   * @param shooter The shooter subsystem to run.
   * @param power The power level to set the shooter motor to, typically between -1.0 and 1.0.
   */
  public ShootingOnlyCommand(Shooter shooter, double power) {
    this.shooter = shooter;
    this.power = power;
    addRequirements(shooter);
  }

  /**
   * Stop the shooter motors.
   */
  @Override
  public void initialize() {
    shooter.shoot(0);
  }

  /**
   * Runs the shooter at the specified power level.
   */
  @Override
  public void execute() {
    shooter.shoot(power);
  }

  /**
   * Stop the shooter motors.
   */
  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
