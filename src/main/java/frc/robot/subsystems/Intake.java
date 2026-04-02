// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;

/**
 * Retractable intake subsystem. Deploys and then intakes using a single motor.
 * Feeds into Spindexer, which feeds into Shooter.
 */
public class Intake extends SubsystemBase {
  SparkFlex intakeMotor = new SparkFlex(24, MotorType.kBrushless);
  SparkMax deployMotor = new SparkMax(27, MotorType.kBrushless);
  double deployAbsoluteEncoder = deployMotor.getAbsoluteEncoder().getPosition();
  double lastPower;

  /**
   * Nothing done in constructor.
   */
  public Intake() {
    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      SmartDashboard.putNumber("INT - Deploy PID-P", 0.2);
      SmartDashboard.putNumber("INT - Deploy PID-I", 0.0);
      SmartDashboard.putNumber("INT - Deploy PID-D", 0.004);
    }
    SmartDashboard.putNumber("Intake V", 0.0);
    SmartDashboard.putNumber("IntakeDeploy V", 0.0);
    lastPower = 0.0;
  }

  /**
   * Clamps PID values to reasonable ranges.
   * P: 0.0 - 0.5, I: 0.0 - 0.01, D: 0.0 - 0.05
   */
  private void clampPIDValues() {
    double p = SmartDashboard.getNumber("INT - Deploy PID-P", 0.2);
    double i = SmartDashboard.getNumber("INT - Deploy PID-I", 0.0);
    double d = SmartDashboard.getNumber("INT - Deploy PID-D", 0.004);

    p = Math.max(0.0, Math.min(0.5, p));
    i = Math.max(0.0, Math.min(0.01, i));
    d = Math.max(0.0, Math.min(0.05, d));

    SmartDashboard.putNumber("INT - Deploy PID-P", p);
    SmartDashboard.putNumber("INT - Deploy PID-I", i);
    SmartDashboard.putNumber("INT - Deploy PID-D", d);
  }

  /**
   * Deploys the intake by setting the deploy motor to a specified power level.
   * 
   * @param power The power level to set the deploy motor to, typically between
   *              -1.0 and 1.0.
   */
  public void intake(double power) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      double dashboardPower = SmartDashboard.getNumber("Intake V", 0.0);
      if (dashboardPower != 0) {
        power = dashboardPower;
      }
    }
    if (lastPower != power) {
      intakeMotor.set(-power);
      lastPower = power;
    }

  }

  /**
   * Deploys the intake by setting the deploy motor to a specified power level.
   * 
   * @param power The power level to set the deploy motor to, typically between
   *              -1.0 and 1.0.
   */
  public void deploy(double power) {
    // When limits are needed on position, check last years code for reference.
    // Needs PID.
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      double dashboardPower = SmartDashboard.getNumber("IntakeDeploy V", 0.0);
      if (dashboardPower != 0) {
        power = dashboardPower;
      }
    }
    /*
     * Tune location numbers
     * if(power<=0){
     * if(getPos() <= 0){
     * deployMotor.set(-power);
     * }
     * else{
     * deployMotor.set(0);
     * }
     * }
     * else{
     * if(getPos() > -20.7){
     * deployMotor.set(-power);
     * }
     * else{
     * deployMotor.set(0);
     * }
     * }
     */
    deployMotor.set(power);
  }

  /**
   * Get absolute encoder position
   * 
   * @return Absolute Encoder Position of the motor.
   */
  public double getPos() {
    return deployAbsoluteEncoder;
  }

  /**
   * Periodically updates the SmartDashboard with the current position
   * of the intake and the target position for deployment.
   */
  @Override
  public void periodic() {
    deployAbsoluteEncoder = deployMotor.getAbsoluteEncoder().getPosition();
    SmartDashboard.putNumber("INT - Deploy Target", Globals.targetPos.intakeDeployTarget);
    SmartDashboard.putNumber("INT - Deploy Position", deployAbsoluteEncoder);
    SmartDashboard.putNumber("INT - Motor Power", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("INT - Deploy Power", deployMotor.getAppliedOutput());
    
    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      clampPIDValues();
    }
  }
}