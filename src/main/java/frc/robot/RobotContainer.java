// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbOnlyCommand;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.commands.Autonomous.ContinuousRotateToAngle;
import frc.robot.commands.Intaking.IntakeOnlyCommand;
import frc.robot.commands.Intaking.PositionIntakeCommand;
import frc.robot.commands.Shooting.FeedRollOnly;
import frc.robot.commands.Shooting.HoodAngleOnlyCommand;
import frc.robot.commands.Shooting.HoodAnglerPositionCommand;
import frc.robot.commands.Shooting.ShootingOnlyCommand;
import frc.robot.commands.Shooting.ShootingRPMCommand;
import frc.robot.commands.Shooting.SpindexOnlyCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EagleEye;
import frc.robot.subsystems.FeedRoller;
import frc.robot.subsystems.HoodAngler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import java.util.function.DoubleSupplier;

public class RobotContainer {
    /**<----------Drivetrain---------->*/
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


    /**<----------Controllers---------->*/
    private CommandXboxController buttonsXbox = new CommandXboxController(2);
    private CommandXboxController driverXbox;
    private Joystick rightJoystick;
    private Joystick leftJoystick;

    /**<----------Subsystems---------->*/
    private final EagleEye eagleEye;
    private final HoodAngler hoodAngler = new HoodAngler();
    private final Intake intake =  new Intake();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Spindexer spindexer = new Spindexer();
    private final FeedRoller feedRoller = new FeedRoller();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final EagleEyeCommand eagleEyeCommand;
    private final ContinuousRotateToAngle autoRotate;
    private final SendableChooser<Command> auto;

    /**<----------Autonomous Commands---------->*/

    // Far-Shot Auto
    private final ShootingRPMCommand shootingFarShotAuto = new ShootingRPMCommand(shooter, 5146);
    private final HoodAnglerPositionCommand hoodAnglerFarShotAuto = new HoodAnglerPositionCommand(hoodAngler, 0); 
    private final FeedRollOnly feedRollFarShotAuto = new FeedRollOnly(feedRoller, 1);
    private final SpindexOnlyCommand spindexOnlyFarShotAuto = new SpindexOnlyCommand(spindexer, 0.5); 
    private final ParallelCommandGroup farShotCommandAuto = new ParallelCommandGroup(
        shootingFarShotAuto.withTimeout(5), 
        hoodAnglerFarShotAuto.withTimeout(5),
        feedRollFarShotAuto.withTimeout(5), 
        spindexOnlyFarShotAuto.withTimeout(5)
    );

    // Mid-Shot Auto
    private final ShootingRPMCommand shootingMidShotAuto = new ShootingRPMCommand(shooter, 5146);
    private final HoodAnglerPositionCommand hoodAnglerMidShotAuto = new HoodAnglerPositionCommand(hoodAngler, 0); 
    private final FeedRollOnly feedRollMidShotAuto = new FeedRollOnly(feedRoller, 1);
    private final SpindexOnlyCommand spindexOnlyMidShotAuto = new SpindexOnlyCommand(spindexer, 0.5); 
    private final ParallelCommandGroup midShotCommandAuto = new ParallelCommandGroup(
        shootingMidShotAuto.withTimeout(5), 
        hoodAnglerMidShotAuto.withTimeout(5),
        feedRollMidShotAuto.withTimeout(5), 
        spindexOnlyMidShotAuto.withTimeout(5)
    );

    // Auto Shoot Auto

    // Intake Auto

    // Climb Auto
    private final ClimbOnlyCommand climbUpCommandAuto = new ClimbOnlyCommand(climber, 0.5);
    private final ClimbOnlyCommand climbDownCommandAuto = new ClimbOnlyCommand(climber, -0.5);
    private final SequentialCommandGroup climbCommandAuto = new SequentialCommandGroup(
        climbUpCommandAuto.withTimeout(5), 
        climbDownCommandAuto.withTimeout(3)
    );



    /**<----------Teleop Commands---------->*/

    // Mid-Shot
    private final ShootingRPMCommand shootingMidShot = new ShootingRPMCommand(shooter, 4814);
    private final HoodAnglerPositionCommand hoodAngleMidShot = new HoodAnglerPositionCommand(hoodAngler, 0 /*who knows */);
    private final ParallelCommandGroup midShotCommand = new ParallelCommandGroup(
        shootingMidShot, 
        hoodAngleMidShot
    );

    // Far-Shot
    private final ShootingRPMCommand shootingFarShot = new ShootingRPMCommand(shooter, 5146);
    private final HoodAnglerPositionCommand hoodAngleFarShot = new HoodAnglerPositionCommand(hoodAngler, 0 /*who knows */);
    private final ParallelCommandGroup farShotCommand = new ParallelCommandGroup(
        shootingFarShot, 
        hoodAngleFarShot
    );

    // Mid-Pass
    private final ShootingRPMCommand shootingMidPass = new ShootingRPMCommand(shooter, 5644);
    private final HoodAnglerPositionCommand hoodAngleMidPass = new HoodAnglerPositionCommand(hoodAngler, -600 /*who knows */);
    private final ParallelCommandGroup midPassCommand = new ParallelCommandGroup(
        shootingMidPass, 
        hoodAngleMidPass
    );

    // Far-Pass
    private final ShootingRPMCommand shootingFarPass = new ShootingRPMCommand(shooter, 10000);
    private final HoodAnglerPositionCommand hoodAngleFarPass = new HoodAnglerPositionCommand(hoodAngler, -900 /*who knows */);
    private final ParallelCommandGroup farPassCommand = new ParallelCommandGroup(shootingFarPass, hoodAngleFarPass);

    // Hood Angle Only
    private final HoodAngleOnlyCommand hoodAngleOnlyCommandUp = new HoodAngleOnlyCommand(hoodAngler, 0.1);
    private final HoodAngleOnlyCommand hoodAngleOnlyCommandDown = new HoodAngleOnlyCommand(hoodAngler, -0.1);

    // Hood Angle Positioning Commands
    private final HoodAnglerPositionCommand hoodAngleLowPosition = new HoodAnglerPositionCommand(hoodAngler, -1050);
    private final HoodAnglerPositionCommand hoodAngleMiddlePosition = new HoodAnglerPositionCommand(hoodAngler, -650);
    private final HoodAnglerPositionCommand hoodAngleHighPosition = new HoodAnglerPositionCommand(hoodAngler, -100);

    // Intaking / Outtaking
    private final IntakeOnlyCommand intakeOnlyCommand = new IntakeOnlyCommand(intake, 0.8);
    private final SequentialCommandGroup intakeRampDown = new IntakeOnlyCommand(intake, 0.25).withTimeout(0.25).andThen(
        new IntakeOnlyCommand(intake, 0.1).withTimeout(0.25)
    );
    
    private final IntakeOnlyCommand outtakeOnlyCommand = new IntakeOnlyCommand(intake, -0.8);
    private final SequentialCommandGroup outtakeRampDown = new IntakeOnlyCommand(intake, -0.25).withTimeout(0.25).andThen(
        new IntakeOnlyCommand(intake, -0.1).withTimeout(0.25)
    );

    private final PositionIntakeCommand deployIntake = new PositionIntakeCommand(intake, -0.1);
    private final PositionIntakeCommand deployIntakeDown = new PositionIntakeCommand(intake, 0.1);

    // Shooting
    private final ShootingOnlyCommand shootingOnlyCommand = new ShootingOnlyCommand(shooter, .75);
    private final SequentialCommandGroup shootingRampDown = new ShootingOnlyCommand(shooter, 0.5).withTimeout(.25).andThen(
        new ShootingOnlyCommand(shooter, .25).withTimeout(.25).andThen(
        new ShootingOnlyCommand(shooter, .1).withTimeout(.1))
    );

    // Climb
    private final ClimbOnlyCommand climbOnlyCommandUp = new ClimbOnlyCommand(climber, 0.5);
    private final ClimbOnlyCommand climbOnlyCommandDown = new ClimbOnlyCommand(climber, -0.5);

    // Spindex 
    private final SpindexOnlyCommand spindexOnlyCommandShoot = new SpindexOnlyCommand(spindexer, 0.3);
    private final SequentialCommandGroup spindexRampDownShoot = new SpindexOnlyCommand(spindexer, 0.25).withTimeout(0.25).andThen(
        new SpindexOnlyCommand(spindexer, 0.1).withTimeout(0.25)
    );

    private final SpindexOnlyCommand spindexOnlyCommandIntake = new SpindexOnlyCommand(spindexer, 0.5);
    private final SequentialCommandGroup spindexRampDownIntake = new SpindexOnlyCommand(spindexer, 0.25).withTimeout(0.25).andThen(
        new SpindexOnlyCommand(spindexer, 0.1).withTimeout(0.25)
    );

    // Feed Roller
    private final FeedRollOnly feedRollOnlyCommand = new FeedRollOnly(feedRoller, 1);
    private final SequentialCommandGroup feedRollRampDown = new FeedRollOnly(feedRoller, 0.75).withTimeout(0.25).andThen(
        new FeedRollOnly(feedRoller, 0.5).withTimeout(0.25).andThen(
        new FeedRollOnly(feedRoller, 0.25).withTimeout(0.25).andThen(
        new FeedRollOnly(feedRoller, 0.1).withTimeout(0.25)))
    );

    /**<----------Path follower---------->*/
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Far Shot (No Movement)");
        SmartDashboard.putData("Auto Mode", autoChooser);

        NamedCommands.registerCommand("Far Shot", farShotCommandAuto);
        NamedCommands.registerCommand("Mid Shot", midShotCommandAuto);
        NamedCommands.registerCommand("Climb", climbCommandAuto);

        if (Constants.EagleEyeConstants.EAGLEEYE_ENABLED) {
            eagleEye = new EagleEye();
            eagleEyeCommand = new EagleEyeCommand(eagleEye);
        } else {
            eagleEye = null;
            eagleEyeCommand = null;
        }

        autoRotate = new ContinuousRotateToAngle(drivetrain, 
        (DoubleSupplier) () -> MathUtil.applyDeadband(-leftJoystick.getY() * MaxSpeed, OperatorConstants.TRANSLATION_DEADBAND),
        (DoubleSupplier) () -> MathUtil.applyDeadband(-leftJoystick.getX() * MaxSpeed, OperatorConstants.ROTATION_DEADBAND));

        // Connect the controllers before binding
        if (Constants.OperatorConstants.XBOX_DRIVE) {
            driverXbox = new CommandXboxController(0);
            if (DriverStation.isJoystickConnected(1)) {
                buttonsXbox = new CommandXboxController(1);
            } else {
                buttonsXbox = driverXbox;
            }
            rightJoystick = null; // Not used in Xbox mode
            leftJoystick = null; // Not used in Xbox mode
        } else {
            rightJoystick = new Joystick(0);
            leftJoystick = new Joystick(1);
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
        auto = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Choose Auto", auto);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            OperatorConstants.XBOX_DRIVE ? drivetrain.applyRequest(() ->
                drive.withVelocityX(driverXbox.getLeftX() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXbox.getLeftY() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ) : drivetrain.applyRequest(() ->
                drive.withVelocityX(-leftJoystick.getX() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftJoystick.getY() * MaxSpeed) // Drive left with negative X (left)
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
        } else {
            new JoystickButton(leftJoystick, 4).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        }
        

        // Right Bumper Button - Final Shooting
        buttonsXbox.rightBumper().whileTrue(new ParallelCommandGroup(
            feedRollOnlyCommand, 
            spindexOnlyCommandShoot)
        ); 
        buttonsXbox.rightBumper().onFalse(new ParallelCommandGroup(
            feedRollRampDown, 
            spindexRampDownShoot)
        ); 

        // Left Bumper Button - Intaking
        buttonsXbox.leftBumper().whileTrue(new ParallelCommandGroup(
            intakeOnlyCommand, 
            spindexOnlyCommandIntake)
        );
        buttonsXbox.leftBumper().onFalse(new ParallelCommandGroup(
            intakeRampDown, 
            spindexRampDownIntake)
        );

        // Menu Button - Outtaking
        buttonsXbox.button(7).whileTrue(outtakeOnlyCommand); 
        buttonsXbox.button(7).onFalse(outtakeRampDown);

        // DPad Up - Climb Up
        buttonsXbox.pov(0).whileTrue(climbOnlyCommandUp); 
        // DPad Down - Climb Down
        buttonsXbox.pov(180).whileTrue(climbOnlyCommandDown); 

        // DPad Right - Hood Angle Up
        buttonsXbox.pov(90).whileTrue(deployIntakeDown);
        // DPad Left - Hood Angle Down
        buttonsXbox.pov(270).whileTrue(deployIntake);
        
        // A Button - Far Pass
        buttonsXbox.a().whileTrue(farPassCommand); 
        buttonsXbox.a().onFalse(shootingRampDown);
        // B Button - Mid Pass
        buttonsXbox.b().whileTrue(midPassCommand); 
        buttonsXbox.b().onFalse(shootingRampDown);
        // X Button - Far Shot
        buttonsXbox.x().whileTrue(farShotCommand); 
        buttonsXbox.x().onFalse(shootingRampDown);
        // Y Button - Mid Shot
        buttonsXbox.y().whileTrue(midShotCommand);
        buttonsXbox.y().onFalse(shootingRampDown);
    
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}