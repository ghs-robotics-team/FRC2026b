// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.SequenceInputStream;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.commands.HoodAnglerPositionCommand;
import frc.robot.commands.IntakeOnlyCommand;
import frc.robot.commands.PositionIntakeCommand;
import frc.robot.commands.ShootingOnlyCommand;
import frc.robot.commands.SpindexOnlyCommand;
import frc.robot.commands.ShootingRPMCommand;

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
    private XboxController buttonsXbox = new XboxController(2);
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
    private final ShootingRPMCommand shootingMidShot = new ShootingRPMCommand(shooter, 2975);
    private final HoodAnglerPositionCommand hoodAngleMidShot = new HoodAnglerPositionCommand(hoodAngler, 0 /*who knows */);
    private final ParallelCommandGroup midShotCommand = new ParallelCommandGroup(shootingMidShot, hoodAngleMidShot);

    private final ShootingRPMCommand shootingFarShot = new ShootingRPMCommand(shooter, 3215);
    private final HoodAnglerPositionCommand hoodAngleFarShot = new HoodAnglerPositionCommand(hoodAngler, -300 /*who knows */);
    private final ParallelCommandGroup farShotCommand = new ParallelCommandGroup(shootingFarShot, hoodAngleFarShot);

    private final ShootingRPMCommand shootingMidPass = new ShootingRPMCommand(shooter, 3420);
    private final HoodAnglerPositionCommand hoodAngleMidPass = new HoodAnglerPositionCommand(hoodAngler, -600 /*who knows */);
    private final ParallelCommandGroup midPassCommand = new ParallelCommandGroup(shootingMidPass, hoodAngleMidPass);

    private final ShootingRPMCommand shootingFarPass = new ShootingRPMCommand(shooter, 5130);
    private final HoodAnglerPositionCommand hoodAngleFarPass = new HoodAnglerPositionCommand(hoodAngler, -900 /*who knows */);
    private final ParallelCommandGroup farPassCommand = new ParallelCommandGroup(shootingFarPass, hoodAngleFarPass);


    private final HoodAngleOnlyCommand hoodAngleOnlyCommandUp = new HoodAngleOnlyCommand(hoodAngler, 0.1);
    private final HoodAngleOnlyCommand hoodAngleOnlyCommandDown = new HoodAngleOnlyCommand(hoodAngler, -0.1);

    private final HoodAnglerPositionCommand hoodAngleLowPosition = new HoodAnglerPositionCommand(hoodAngler, -1050);
    private final HoodAnglerPositionCommand hoodAngleMiddlePosition = new HoodAnglerPositionCommand(hoodAngler, -650);
    private final HoodAnglerPositionCommand hoodAngleHighPosition = new HoodAnglerPositionCommand(hoodAngler, -100);

    private final IntakeOnlyCommand intakeOnlyCommand = new IntakeOnlyCommand(intake, 0.8);
    private final SequentialCommandGroup intakeRampDown = new IntakeOnlyCommand(intake, 0.25).withTimeout(0.25).andThen(
        new IntakeOnlyCommand(intake, 0.1).withTimeout(0.25));

    private final IntakeOnlyCommand outtakeOnlyCommand = new IntakeOnlyCommand(intake, -0.8);
    private final SequentialCommandGroup outtakeRampDown = new IntakeOnlyCommand(intake, -0.25).withTimeout(0.25).andThen(
        new IntakeOnlyCommand(intake, -0.1).withTimeout(0.25));

    private final ShootingOnlyCommand shootingOnlyCommand = new ShootingOnlyCommand(shooter, .75);
    private final SequentialCommandGroup shootingRampDown = new ShootingOnlyCommand(shooter, 0.5).withTimeout(.25).andThen(
        new ShootingOnlyCommand(shooter, .25).withTimeout(.25).andThen(
            new ShootingOnlyCommand(shooter, .1).withTimeout(.1)));

    private final ClimbOnlyCommand climbOnlyCommandUp = new ClimbOnlyCommand(climber, 0.5);
    private final ClimbOnlyCommand climbOnlyCommandDown = new ClimbOnlyCommand(climber, -0.5);

    private final SpindexOnlyCommand spindexOnlyCommandShoot = new SpindexOnlyCommand(spindexer, 0.5);
    private final SequentialCommandGroup spindexRampDownShoot = new SpindexOnlyCommand(spindexer, 0.25).withTimeout(0.25).andThen(
        new SpindexOnlyCommand(spindexer, 0.1).withTimeout(0.25));

    private final SpindexOnlyCommand spindexOnlyCommandIntake = new SpindexOnlyCommand(spindexer, 0.5);
    private final SequentialCommandGroup spindexRampDownIntake = new SpindexOnlyCommand(spindexer, 0.25).withTimeout(0.25).andThen(
        new SpindexOnlyCommand(spindexer, 0.1).withTimeout(0.25));

    

    private final FeedRollOnly feedRollOnlyCommand = new FeedRollOnly(feedRoller, 1);
    private final SequentialCommandGroup feedRollRampDown = new FeedRollOnly(feedRoller, 0.75).withTimeout(0.25).andThen(
        new FeedRollOnly(feedRoller, 0.5).withTimeout(0.25).andThen(
            new FeedRollOnly(feedRoller, 0.25).withTimeout(0.25).andThen(
                new FeedRollOnly(feedRoller, 0.1).withTimeout(0.25))));

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
            /**driverXbox = new XboxController(0);
            if (DriverStation.isJoystickConnected(1)) {
                buttonsXbox = new XboxController(1);
            } else {
                buttonsXbox = driverXbox;
            }*/
            rightJoystick = null; // Not used in Xbox mode
            leftJoystick = null; // Not used in Xbox mode
        } else {
            rightJoystick = new Joystick(0);
            leftJoystick = new Joystick(1);
            //buttonsXbox = new XboxController(2);
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
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
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
            * Xbox bindings
            * +------------------------------+-------------------------------+
            * | Control                      | Action                        |
            * +------------------------------+-------------------------------+
            * | Driver Xbox start            | Reset field direction         |
            * +------------------------------+-------------------------------+
            */
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

            new JoystickButton(buttonsXbox, 6).whileTrue(new ParallelCommandGroup(feedRollOnlyCommand, spindexOnlyCommandShoot)); // Right Bumper Button
            //buttonsXbox.rightBumper().whileTrue(new ParallelCommandGroup(feedRollOnlyCommand, spindexOnlyCommandShoot));
            //buttonsXbox.rightBumper().onFalse(new ParallelCommandGroup(feedRollRampDown, spindexRampDownShoot));
            new JoystickButton(buttonsXbox, 6).onFalse(new ParallelCommandGroup(feedRollRampDown, spindexRampDownShoot)); 
            //new JoystickButton(buttonsXbox, 6).whileTrue(shootingRPMCommand);


            new JoystickButton(buttonsXbox, 5).whileTrue(new ParallelCommandGroup(intakeOnlyCommand, spindexOnlyCommandIntake));
            new JoystickButton(buttonsXbox, 5).onFalse(new ParallelCommandGroup(intakeRampDown, spindexRampDownIntake)); // Left Bumper Button

            new JoystickButton(buttonsXbox, 7).whileTrue(outtakeOnlyCommand); // Menu
            new JoystickButton(buttonsXbox, 7).onFalse(outtakeRampDown);

            new POVButton(buttonsXbox, 0).whileTrue(climbOnlyCommandUp); // DPad Up
            new POVButton(buttonsXbox, 180).whileTrue(climbOnlyCommandDown); // DPad Down

            new POVButton(buttonsXbox, 90).whileTrue(deployIntake);
            new POVButton(buttonsXbox, 270).whileTrue(deployIntakeDown);

            new JoystickButton(buttonsXbox, 1).whileTrue(farPassCommand); // A
            new JoystickButton(buttonsXbox, 2).whileTrue(midPassCommand); // B
            new JoystickButton(buttonsXbox, 3).whileTrue(farShotCommand); // X
            new JoystickButton(buttonsXbox, 4).whileTrue(midShotCommand); // Y
            new JoystickButton(buttonsXbox, 1).onFalse(shootingRampDown);
            new JoystickButton(buttonsXbox, 2).onFalse(shootingRampDown);
            new JoystickButton(buttonsXbox, 3).onFalse(shootingRampDown);
            new JoystickButton(buttonsXbox, 4).onFalse(shootingRampDown);

            //buttonsXbox.x().whileTrue(shootingFarShot);
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
