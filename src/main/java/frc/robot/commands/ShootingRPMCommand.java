// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootingRPMCommand extends Command {
  /** Creates a new ShootingRPMCommand. */
  Shooter shooter;
  double rpm;

  /**
   * Creates a new ShootingOnlyCommand.
   * @param shooter The shooter subsystem to run.
   * @param power The power level to set the shooter motor to, typically between -1.0 and 1.0.
   */
  public ShootingRPMCommand(Shooter shooter, double rpm) {
    this.shooter = shooter;
    this.rpm = rpm;
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
    shooter.shootTargetSpeed(rpm);
  }

  /**
   * Stop the shooter motors.
   */
  @Override
  public void end(boolean interrupted) {
    shooter.shootTargetSpeed(0);
  }
  public boolean isFinished() {
    return false;
  }
}
