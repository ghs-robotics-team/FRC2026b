// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Spindexer subsystem for indexing and feeding game pieces to the feed roller and shooter.
 */
public class Spindexer extends SubsystemBase {
  SparkFlex indexer = new SparkFlex(4, MotorType.kBrushless);

  /**
   * Nothing done in constructor.
   */
  public Spindexer() {}

  /**
   * Sets the power level of the indexer motor to run the spindexer.
   * @param power The power level to set the indexer motor to, typically between -1.0 and 1.0.
   */
  public void run(double power) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      power = SmartDashboard.getNumber("Spindexer V", 0.1);
    }
    indexer.set(-power);
  }

  /**
   * Periodic is not used.
   */
  @Override
  public void periodic() {
  }
}
