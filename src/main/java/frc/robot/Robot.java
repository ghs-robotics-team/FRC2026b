// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.EagleEyeConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    public double getEagleEyeConfidence(LimelightHelpers.PoseEstimate pose, SwerveDriveState driveState) {
        if (pose.tagCount < 1) {
            return 0.0;
        }

        if (pose.avgTagDist > EagleEyeConstants.MAX_TAG_DISTANCE_METERS)
            return 0.0;
        if (Math.abs(driveState.Speeds.omegaRadiansPerSecond) > EagleEyeConstants.MAX_ROTATION_VELOCITY)
            return 0.0;
        if (Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond) > EagleEyeConstants.MAX_VISION_SPEED)
            return 0.0;

        // Start with full trust
        double confidence = 1.0;

        // Distance Penalties
        double distMeters = pose.avgTagDist;
        double distFactor = MathUtil.clamp(1.0 - distMeters / EagleEyeConstants.MAX_TAG_DISTANCE_METERS, 0.0, 1.0);
        confidence *= distFactor;

        // Tag Count Bonuses
        if (pose.tagCount >= 2) {
            confidence *= 1.0; // keep
        } else {
            confidence *= 0.75; // single-tag penalty
        }

        // Disabled or stationary bonus
        if (DriverStation.isDisabled()) {
            confidence *= 1.2;
        }

         // Clamp final confidence to [0, 1] to avoid invalid values.
        confidence = MathUtil.clamp(confidence, 0.0, 1.0);

        return confidence;
    }

    private boolean updateVisionFromEagleEye(String cameraName, double headingDeg, SwerveDriveState driveState) {
        boolean updated = false;

        // Update Limelight heading to match drivetrain heading, so that the pose estimate is in the correct coordinate space.
        LimelightHelpers.SetRobotOrientation(cameraName, headingDeg, 0, 0, 0, 0, 0);
        
        // Get the latest pose estimate from Limelight. This will return null if no targets are detected.
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        
        if (measurement != null) {
            String prefix = cameraName.endsWith("b") ? "EEB" : "EEA";
            SmartDashboard.putNumber(prefix + " NumTags", measurement.tagCount);
            SmartDashboard.putNumber(prefix + " Avg Tag Dist", measurement.avgTagDist);
            SmartDashboard.putNumber(prefix + " Rotation Vel", driveState.Speeds.omegaRadiansPerSecond);
            SmartDashboard.putNumber(prefix + " Total Vel", Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond));

            double confidence = getEagleEyeConfidence(measurement, driveState);
            SmartDashboard.putNumber(prefix + " Confidence", confidence);
            if (confidence >= EagleEyeConstants.MIN_CONFIDENCE) {
                double scale = 1.0 / Math.max(confidence, 0.1);
                var v = VecBuilder.fill(
                    EagleEyeConstants.X_SIGMA * scale,
                    EagleEyeConstants.Y_SIGMA * scale,
                    EagleEyeConstants.THETA_SIGMA * scale
                );
                m_robotContainer.drivetrain.addVisionMeasurement(measurement.pose, measurement.timestampSeconds, v);
                updated = true;
            }
        }

        return updated;
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (EagleEyeConstants.EAGLEEYE_ENABLED) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            boolean fused = false;
            fused |= updateVisionFromEagleEye("limelight-cama", headingDeg, driveState);
            if (fused) {
                SmartDashboard.putBoolean("SSA Eagleeye Read", true);
            }

            fused |= updateVisionFromEagleEye("limelight-camb", headingDeg, driveState);
            if (fused) {
                SmartDashboard.putBoolean("SSA Eagleeye Read", true);
            }

            if (!fused) {
                SmartDashboard.putBoolean("SS Eagleeye Read", false);
            } else {
                SmartDashboard.putBoolean("SS Eagleeye Read", true);
            }
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
