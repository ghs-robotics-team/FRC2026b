// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EagleEye;
import frc.robot.subsystems.FeedRoller;
import frc.robot.subsystems.HoodAngler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbOnlyCommand;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.commands.FeedRollOnly;
import frc.robot.commands.HoodAngleOnlyCommand;
import frc.robot.commands.IntakeOnlyCommand;
import frc.robot.commands.PositionIntakeCommand;
import frc.robot.commands.ShootingOnlyCommand;
import frc.robot.commands.SpindexOnlyCommand;

public class RobotContainer {
    private double MaxSpeed = Constants.OperatorConstants.MAX_SPEED; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Constants.OperatorConstants.MAX_ANGULAR_SPEED; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * OperatorConstants.TRANSLATION_DEADBAND).withRotationalDeadband(MaxAngularRate * OperatorConstants.ROTATION_DEADBAND) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private XboxController buttonsXbox;
    private XboxController driverXbox;
    private Joystick rightJoystick;
    private Joystick leftJoystick;

    // Subsystems
    private final EagleEye eagleEye;
    private final HoodAngler hoodAngler = new HoodAngler();
    private final Intake intake =  new Intake();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Spindexer spindexer = new Spindexer();
    private final FeedRoller feedRoller = new FeedRoller();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final EagleEyeCommand eagleEyeCommand;

    // Basic Teleop Commands
    private final HoodAngleOnlyCommand hoodAngleOnlyCommandUp = new HoodAngleOnlyCommand(hoodAngler, 0.1);
    private final HoodAngleOnlyCommand hoodAngleOnlyCommandDown = new HoodAngleOnlyCommand(hoodAngler, -0.1);
    private final IntakeOnlyCommand intakeOnlyCommand = new IntakeOnlyCommand(intake, 0.1);
    private final IntakeOnlyCommand outtakeOnlyCommand = new IntakeOnlyCommand(intake, -0.1);
    private final ShootingOnlyCommand shootingOnlyCommand = new ShootingOnlyCommand(shooter, 0.1);
    private final ClimbOnlyCommand climbOnlyCommandUp = new ClimbOnlyCommand(climber, 0.1);
    private final ClimbOnlyCommand climbOnlyCommandDown = new ClimbOnlyCommand(climber, -0.1);
    private final SpindexOnlyCommand spindexOnlyCommand = new SpindexOnlyCommand(spindexer, 0.1);
    private final FeedRollOnly feedRollOnly = new FeedRollOnly(feedRoller, 0.1);
    private final PositionIntakeCommand deployIntake = new PositionIntakeCommand(intake, -0.1);
    private final PositionIntakeCommand deployIntakeDown = new PositionIntakeCommand(intake, 0.1);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        if (Constants.EagleEyeConstants.EAGLEEYE_ENABLED) {
            eagleEye = new EagleEye();
            eagleEyeCommand = new EagleEyeCommand(eagleEye);
        } else {
            eagleEye = null;
            eagleEyeCommand = null;
        }

        // Connect the controllers before binding
        if (Constants.OperatorConstants.XBOX_DRIVE) {
            driverXbox = new XboxController(0);
            if (DriverStation.isJoystickConnected(1)) {
                buttonsXbox = new XboxController(1);
            } else {
                buttonsXbox = driverXbox;
            }
            rightJoystick = null; // Not used in Xbox mode
            leftJoystick = null; // Not used in Xbox mode
        } else {
            rightJoystick = new Joystick(0);
            leftJoystick = new Joystick(1);
            buttonsXbox = new XboxController(2);
            driverXbox = null; // Not used in joystick mode
        }

        configureBindings();

        // Set default commands
        if (Constants.EagleEyeConstants.EAGLEEYE_ENABLED)
        {
            eagleEye.setDefaultCommand(eagleEyeCommand);
        }

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            OperatorConstants.XBOX_DRIVE ? drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ) : drivetrain.applyRequest(() ->
                drive.withVelocityX(-leftJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rightJoystick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        if (OperatorConstants.XBOX_DRIVE) {
            /*
            * Joystick bindings
            * +------------------------------+-------------------------------+
            * | Control                      | Action                        |
            * +------------------------------+-------------------------------+
            * | Driver Xbox start            | Reset field direction         |
            * +------------------------------+-------------------------------+
            */
            new JoystickButton(driverXbox, 7).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        } else {
            /*
            * Joystick bindings
            * +------------------------------+-------------------------------+
            * | Control                      | Action                        |
            * +------------------------------+-------------------------------+
            * | Left Joystick Button 4       | Reset field direction         |
            * +------------------------------+-------------------------------+
            */
            new JoystickButton(leftJoystick, 4).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
            new JoystickButton(buttonsXbox, 8).whileTrue(shootingOnlyCommand); // Right Stick Button
            new JoystickButton(buttonsXbox, 1).whileTrue(intakeOnlyCommand); // A

            //new JoystickButton(buttonsXbox, 4).whileTrue(deployIntake); // Y

            new POVButton(buttonsXbox, 0).whileTrue(climbOnlyCommandUp); // DPad Up
            new POVButton(buttonsXbox, 180).whileTrue(climbOnlyCommandDown); // DPad Down

            new POVButton(buttonsXbox, 90).whileTrue(hoodAngleOnlyCommandUp); // DPad Right
            new POVButton(buttonsXbox, 270).whileTrue(hoodAngleOnlyCommandDown); // DPad Left

            new JoystickButton(buttonsXbox, 3).whileTrue(spindexOnlyCommand); // X

            new JoystickButton(buttonsXbox, 8).whileTrue(shootingOnlyCommand); // Right Stick Button
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
