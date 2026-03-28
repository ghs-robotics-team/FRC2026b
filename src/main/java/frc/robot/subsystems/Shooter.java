// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shoots game pieces at end of subsystem chain.
 * Uses a single motor to shoot and a hood angler to adjust the angle of the
 * shot.
 */
public class Shooter extends SubsystemBase {
  SparkFlex shooterTop = new SparkFlex(18, MotorType.kBrushless);
  SparkFlex shooterBottom = new SparkFlex(8, MotorType.kBrushless);
  SparkFlexConfig config = new SparkFlexConfig();
  SparkClosedLoopController controllerTop;
  SparkClosedLoopController controllerBottom;
  double lastPower;

  public Shooter() {
    // Configure the internal PID of the motor

    lastPower = 0;

    SmartDashboard.putNumber("SH PID-P", 0.0002);
    SmartDashboard.putNumber("SH PID-I", 0.0);
    SmartDashboard.putNumber("SH PID-D", 0.00);
    SmartDashboard.putNumber("Shooting V", 0.0);
    SmartDashboard.putNumber("Shooting RPM", 0.0);

    config.closedLoop.pid(0.01, 0, 0);

    shooterTop.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    shooterBottom.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Created PID controllers
    controllerTop = shooterTop.getClosedLoopController();
    controllerBottom = shooterBottom.getClosedLoopController();

  }

  /**
   * Sets the power level of the shooter motor to shoot game pieces.
   * 
   * @param power The power level to set the shooter motor to, typically between
   *              -1.0 and 1.0.
   */
  public void shoot(double power) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      double dashboardPower = SmartDashboard.getNumber("Shooting V", 0.0);
      if (dashboardPower != 0) {
        power = dashboardPower;
      }
    }
    if (lastPower != power){
      shooterTop.set(power);
      shooterBottom.set(power);
      lastPower = power;
    }
    
  }

  public void shootTargetSpeed(double rpm) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL) {
      double dashboardRPM = SmartDashboard.getNumber("Shooting RPM", 0.0);
      if (dashboardRPM != 0) {
        rpm = dashboardRPM;
      }
    }

    double P = SmartDashboard.getNumber("SH PID-P", 0.0002);
    double I = SmartDashboard.getNumber("SH PID-I", 0.0);
    double D = SmartDashboard.getNumber("SH PID-D", 0);

    config.closedLoop.pid(P, I, D);

    shooterTop.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    shooterBottom.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    controllerTop.setSetpoint(rpm, ControlType.kVelocity);
    controllerBottom.setSetpoint(rpm, ControlType.kVelocity);
    // SmartDashboard.putNumber("Shooter RPM", rpm);
    // SmartDashboard.putNumber("Shooter Error", config.pid);
  }

  /*
   * Periodic is not used.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("SHO - Top RPM", shooterTop.getEncoder().getVelocity());
    SmartDashboard.putNumber("SHO - Bottom RPM", shooterBottom.getEncoder().getVelocity());
    SmartDashboard.putNumber("SHO - Top Power", shooterTop.getAppliedOutput());
    SmartDashboard.putNumber("SHO - Bottom Power", shooterBottom.getAppliedOutput());
  }
}