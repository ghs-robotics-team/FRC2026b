// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * HoodAngler subsystem for adjusting the angle of the shooter hood.
 * Effecting where the balls are angled when shot.
 */
public class HoodAngler extends SubsystemBase {
  private SparkMax hoodAngler = new SparkMax(14, MotorType.kBrushed);
  private Encoder encoder = new Encoder(1, 0);
  private double hoodEncoderVal = encoder.getRaw();

  /**
   * Nothing done in constructor.
   */
  public HoodAngler() {
    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      SmartDashboard.putNumber("HOO - PID-P", 0.2);
      SmartDashboard.putNumber("HOO - PID-I", 0.0);
      SmartDashboard.putNumber("HOO - PID-D", 0.004);
    }
    SmartDashboard.putNumber("HOO - Target Position", hoodEncoderVal);
  }

  /**
   * Clamps PID values to reasonable ranges.
   * P: 0.0 - 0.5, I: 0.0 - 0.01, D: 0.0 - 0.05
   */
  private void clampPIDValues() {
    double p = SmartDashboard.getNumber("HOO - PID-P", 0.2);
    double i = SmartDashboard.getNumber("HOO - PID-I", 0.0);
    double d = SmartDashboard.getNumber("HOO - PID-D", 0.004);

    p = Math.max(0.0, Math.min(0.5, p));
    i = Math.max(0.0, Math.min(0.01, i));
    d = Math.max(0.0, Math.min(0.05, d));

    SmartDashboard.putNumber("HOO - PID-P", p);
    SmartDashboard.putNumber("HOO - PID-I", i);
    SmartDashboard.putNumber("HOO - PID-D", d);
  }

  /**
   * Sets the power level of the hood angler motor to adjust the angle of the
   * shooter hood.
   * 
   * @param power The power level to set the hood angler motor to, typically
   *              between -1.0 and 1.0.
   */
  public void adjust(double power) {
    if (power <= 0) {
      if (getPos() <= 0) {
        hoodAngler.set(-power);
      } else {
        hoodAngler.set(0);
      }
    } else {
      if (getPos() > -1300) {
        hoodAngler.set(-power);
      } else {
        hoodAngler.set(0);
      }
    }
    hoodAngler.set(power);
  }

  /**
   * Get absolute encoder position
   * 
   * @return Absolute Encoder Position of the motor.
   */
  public double getPos() {
    return hoodEncoderVal;
  }

  /**
   * Get the current hood angle
   * 
   * @return The hood angle in encoder units.
   */
  public double getAngle() {
    return hoodEncoderVal;
  }

  /**
   * Periodically updates the SmartDashboard with the current position of the hood
   * angler.
   */
  @Override
  public void periodic() {
    hoodEncoderVal = encoder.getRaw();
    SmartDashboard.putNumber("HOO - Position", getPos());
    SmartDashboard.putNumber("HOO - Motor Power", hoodAngler.get());
    
    if (Constants.OperatorConstants.WORKSHOP_MODE) {
      clampPIDValues();
    }
  }
}