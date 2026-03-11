// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intaking;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Positions the hood angler to the desired angle, rather than only running the
 * motor.
 */
public class IntakeDeployPositionCommand extends Command {
  private final Intake intake;
  private final double desiredPos;
  private double currentPos;
  private double directionFactor;
  private PIDController pid;

  /**
   * Sets the hood angler subsystem and the desired position.
   * 
   * @param hoodAngler Subsystem to control.
   * @param pos        Position to set the hood angler to, in motor ticks.
   */
  public IntakeDeployPositionCommand(Intake intake, double desiredPos) {
    this.intake = intake;
    this.desiredPos = desiredPos;
    // this.currentPos = SmartDashboard.getNumber("HOOD Pos", 0);
    this.currentPos = intake.getPos();
    this.pid = new PIDController(0.2, 0, 0.004);
    // ADD FEEDFORWARD ONCE K VALUES ARE TESTED.

    SmartDashboard.putNumber("INTAKE-PID-P", pid.getP());
    SmartDashboard.putNumber("INTAKE-PID-I", pid.getI());
    SmartDashboard.putNumber("INTAKE-PID-D", pid.getD());
    addRequirements(intake);
  }

  /**
   * Stops the hood angler from moving.
   */
  @Override
  public void initialize() {
    intake.deploy(0);
    this.currentPos = intake.getPos();
  }

  /**
   * Uses PID to control the hood angler to reach the desired position,
   * while updating SmartDashboard with the current position, direction factor,
   * and PID values.
   */
  @Override
  public void execute() {
    this.currentPos = intake.getPos();

    double P = SmartDashboard.getNumber("INTAKE-PID-P", 0.2);
    double I = SmartDashboard.getNumber("INTAKE-PID-I", 0.0);
    double D = SmartDashboard.getNumber("INTAKE-PID-D", 0.004);

    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    directionFactor = pid.calculate(currentPos, desiredPos);
    directionFactor = MathUtil.clamp(directionFactor, -1, 1);
    SmartDashboard.putNumber("INTAKE DirectionFactor", directionFactor);
    if (pid.getPositionError() > -20 && pid.getPositionError() < 20) {
      // Dead Zone
      intake.deploy(0);
    } else {
      intake.deploy(directionFactor);
    }
  }

  /**
   * Stops the hood angler when the command ends.
   */
  @Override
  public void end(boolean interrupted) {
    intake.deploy(0);
  }

  /**
   * Command ends when the intake is within a certain error range of the target
   * position.
   */
  @Override
  public boolean isFinished() {
    if (pid.getPositionError() > -100 && pid.getPositionError() < 100) {
      // Dead Zone
      return true;
    } else {
      return false;
    }
  }
}