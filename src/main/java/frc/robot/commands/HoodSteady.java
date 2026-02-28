// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.HoodAngler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodSteady extends Command {
  /** Creates a new DeploySteady. */
  HoodAngler hoodAngler;
  PIDController pid;
  public HoodSteady(HoodAngler hoodAngler) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hoodAngler);
    this.pid = new PIDController(0, 0, 0);
    this.hoodAngler = hoodAngler;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get PID Controller direction for elevator to go, find current error from position.
    double direction = pid.calculate(hoodAngler.getPos(), Globals.targetPos.intakeDeployTarget);
    double error = pid.getPositionError();

    if (error > -25 && error < 25) {
      hoodAngler.adjust(0); // deadzone
    } else {
      hoodAngler.adjust(direction); // Move Arm
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
