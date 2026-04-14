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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.commands.Autonomous.ContinuousRotateToAngle;
import frc.robot.commands.Autonomous.ContinuousRotateToAllianceWall;
import frc.robot.commands.Intaking.IntakeOnlyCommand;
import frc.robot.commands.Intaking.PositionIntakeCommand;
import frc.robot.commands.Shooting.FeedRollOnly;
import frc.robot.commands.Shooting.HoodAnglerPositionCommand;
import frc.robot.commands.Shooting.ShootingOnlyCommand;
import frc.robot.commands.Shooting.ShootingRPMCommand;
import frc.robot.commands.Shooting.SpindexOnlyCommand;
import frc.robot.generated.TunerConstants;
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

    /* Other Settings
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    */

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

    private final Spindexer spindexer = new Spindexer();
    private final FeedRoller feedRoller = new FeedRoller();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final EagleEyeCommand eagleEyeCommand;
    @SuppressWarnings("unused")
    private final ContinuousRotateToAngle autoRotate;
    @SuppressWarnings("unused")
    private final ContinuousRotateToAllianceWall rotateToAllianceWall;

    /**<----------Autonomous Commands---------->*/

    // Far-Shot Auto
    private final ShootingRPMCommand shootingFarShotAuto = new ShootingRPMCommand(shooter, 5146);
    private final ShootingRPMCommand shootingFarShotTwoAuto = new ShootingRPMCommand(shooter, 5146);
    private final HoodAnglerPositionCommand hoodAnglerFarShotAuto = new HoodAnglerPositionCommand(hoodAngler, 0);
    private final HoodAnglerPositionCommand hoodAnglerFarShotTwoAuto = new HoodAnglerPositionCommand(hoodAngler, 0); 
    private final FeedRollOnly feedRollFarShotAuto = new FeedRollOnly(feedRoller, 0.5);
    private final SpindexOnlyCommand spindexOnlyFarShotAuto = new SpindexOnlyCommand(spindexer, 0.5); 
    private final ParallelCommandGroup farShotPrepAuto = new ParallelCommandGroup(shootingFarShotTwoAuto, hoodAnglerFarShotTwoAuto);
    private final ParallelCommandGroup farShotAllAuto = new ParallelCommandGroup(
            shootingFarShotAuto, 
            hoodAnglerFarShotAuto, 
            feedRollFarShotAuto, 
            spindexOnlyFarShotAuto
    );

    // Mid-Shot Auto
    private final ShootingRPMCommand shootingMidShotAuto = new ShootingRPMCommand(shooter, 4031);
    private final ShootingRPMCommand shootingMidShotTwoAuto = new ShootingRPMCommand(shooter, 4031);
    private final HoodAnglerPositionCommand hoodAnglerMidShotAuto = new HoodAnglerPositionCommand(hoodAngler, 0);
    private final HoodAnglerPositionCommand hoodAnglerMidShotTwoAuto = new HoodAnglerPositionCommand(hoodAngler, 0);
    private final FeedRollOnly feedRollMidShotAuto = new FeedRollOnly(feedRoller, 0.5);
    private final SpindexOnlyCommand spindexOnlyMidShotAuto = new SpindexOnlyCommand(spindexer, 0.5); 
    private final ParallelCommandGroup midShotPrepAuto = new ParallelCommandGroup(shootingMidShotTwoAuto, hoodAnglerMidShotTwoAuto);
    private final ParallelCommandGroup midShotAllAuto = new ParallelCommandGroup(
            shootingMidShotAuto, 
            hoodAnglerMidShotAuto, 
            feedRollMidShotAuto, 
            spindexOnlyMidShotAuto
    );


    // Auto Shoot Auto (Later)


    // Intake Auto (Later)


    /**<----------Teleop Commands---------->*/

    // Mid-Shot
    private final ShootingRPMCommand shootingMidShot = new ShootingRPMCommand(shooter, 3500);
    private final HoodAnglerPositionCommand hoodAngleMidShot = new HoodAnglerPositionCommand(hoodAngler, 0);
    private final ParallelCommandGroup midShotCommand = new ParallelCommandGroup(
        shootingMidShot, 
        hoodAngleMidShot
    );

    // Far-Shot
    private final ShootingRPMCommand shootingFarShot = new ShootingRPMCommand(shooter, 5146);
    private final HoodAnglerPositionCommand hoodAngleFarShot = new HoodAnglerPositionCommand(hoodAngler, 0);
    private final ParallelCommandGroup farShotCommand = new ParallelCommandGroup(
        shootingFarShot, 
        hoodAngleFarShot
    );

    // Mid-Pass
    private final ShootingRPMCommand shootingMidPass = new ShootingRPMCommand(shooter, 5644);
    private final HoodAnglerPositionCommand hoodAngleMidPass = new HoodAnglerPositionCommand(hoodAngler, -600);
    private final ParallelCommandGroup midPassCommand = new ParallelCommandGroup(
        shootingMidPass, 
        hoodAngleMidPass
    );

    // Far-Pass
    private final ShootingRPMCommand shootingFarPass = new ShootingRPMCommand(shooter, 10000);
    private final HoodAnglerPositionCommand hoodAngleFarPass = new HoodAnglerPositionCommand(hoodAngler, -900);
    private final ParallelCommandGroup farPassCommand = new ParallelCommandGroup(shootingFarPass, hoodAngleFarPass);

    /* 
     Hood Angle Positioning Commands
    private final HoodAnglerPositionCommand hoodAngleLowPosition = new HoodAnglerPositionCommand(hoodAngler, -1050);
    private final HoodAnglerPositionCommand hoodAngleMiddlePosition = new HoodAnglerPositionCommand(hoodAngler, -650);
    private final HoodAnglerPositionCommand hoodAngleHighPosition = new HoodAnglerPositionCommand(hoodAngler, -100);
    */

    // Intaking / Outtaking
    private final IntakeOnlyCommand intakeOnlyCommand = new IntakeOnlyCommand(intake, 0.7);
    private final SequentialCommandGroup intakeRampDown = new IntakeOnlyCommand(intake, 0.25).withTimeout(0.25).andThen(
        new IntakeOnlyCommand(intake, 0.1).withTimeout(0.25)
    );
    
    private final IntakeOnlyCommand outtakeOnlyCommand = new IntakeOnlyCommand(intake, -0.7);
    private final SequentialCommandGroup outtakeRampDown = new IntakeOnlyCommand(intake, -0.25).withTimeout(0.25).andThen(
        new IntakeOnlyCommand(intake, -0.1).withTimeout(0.25)
    );

    private final PositionIntakeCommand deployIntake = new PositionIntakeCommand(intake, -0.1);
    private final PositionIntakeCommand deployIntakeDown = new PositionIntakeCommand(intake, 0.1);

    // Shooting
    private final SequentialCommandGroup shootingRampDown = new ShootingOnlyCommand(shooter, 0.5).withTimeout(.25).andThen(
        new ShootingOnlyCommand(shooter, .25).withTimeout(.25).andThen(
        new ShootingOnlyCommand(shooter, .1).withTimeout(.1))
    );

    // Spindex 
    private final SpindexOnlyCommand spindexOnlyCommandShoot = new SpindexOnlyCommand(spindexer, 0.75);
    private final SequentialCommandGroup spindexRampDownShoot = new SpindexOnlyCommand(spindexer, 0.25).withTimeout(0.25).andThen(
        new SpindexOnlyCommand(spindexer, 0.1).withTimeout(0.25)
    );

    // Feed Roller
    private final FeedRollOnly feedRollOnlyCommand = new FeedRollOnly(feedRoller, 0.5);
    private final SequentialCommandGroup feedRollRampDown = new FeedRollOnly(feedRoller, 0.75).withTimeout(0.25).andThen(
        new FeedRollOnly(feedRoller, 0.5).withTimeout(0.25).andThen(
        new FeedRollOnly(feedRoller, 0.25).withTimeout(0.25).andThen(
        new FeedRollOnly(feedRoller, 0.1).withTimeout(0.25)))
    );

    /**<----------Path follower---------->*/
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Far Shot", farShotPrepAuto.withTimeout(2)
            .andThen(farShotAllAuto).withTimeout(5));
        NamedCommands.registerCommand("Mid Shot", midShotPrepAuto.withTimeout(2)
            .andThen(midShotAllAuto).withTimeout(5));

        autoChooser = AutoBuilder.buildAutoChooser("Far Shot (No Movement)");
        SmartDashboard.putData("Auto Mode", autoChooser);

        if (Constants.EagleEyeConstants.EAGLEEYE_ENABLED) {
            eagleEye = new EagleEye();
            eagleEyeCommand = new EagleEyeCommand(eagleEye);
            eagleEye.setDefaultCommand(eagleEyeCommand);
        } else {
            eagleEye = null;
            eagleEyeCommand = null;
        }

        if (OperatorConstants.XBOX_DRIVE){
            autoRotate = new ContinuousRotateToAngle(drivetrain, 
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(driverXbox.getLeftY(), 2), 
                            -1, 
                            1
                        ) * Math.signum(driverXbox.getLeftY()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed,
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(driverXbox.getLeftX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(driverXbox.getLeftX()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed
                );
                rotateToAllianceWall = new ContinuousRotateToAllianceWall(drivetrain, 
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(driverXbox.getLeftY(), 2), 
                            -1, 
                            1
                        ) * Math.signum(driverXbox.getLeftY()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed,
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(driverXbox.getLeftX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(driverXbox.getLeftX()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed
                );
        } else {
            autoRotate = new ContinuousRotateToAngle(drivetrain, 
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(leftJoystick.getY(), 2), 
                            -1, 
                            1
                        ) * Math.signum(leftJoystick.getY()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed,
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(rightJoystick.getX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(rightJoystick.getX()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed
                );
                rotateToAllianceWall = new ContinuousRotateToAllianceWall(drivetrain, 
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(leftJoystick.getY(), 2), 
                            -1, 
                            1
                        ) * Math.signum(leftJoystick.getY()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed,
                (DoubleSupplier) () -> MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(rightJoystick.getX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(rightJoystick.getX()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed
                );
        }

        drivetrain.getModule(0).getDriveMotor().getDeviceTemp().setUpdateFrequency(4);
        drivetrain.getModule(0).getSteerMotor().getDeviceTemp().setUpdateFrequency(4);

        drivetrain.getModule(1).getDriveMotor().getDeviceTemp().setUpdateFrequency(4);
        drivetrain.getModule(1).getSteerMotor().getDeviceTemp().setUpdateFrequency(4);

        drivetrain.getModule(2).getDriveMotor().getDeviceTemp().setUpdateFrequency(4);
        drivetrain.getModule(2).getSteerMotor().getDeviceTemp().setUpdateFrequency(4);

        drivetrain.getModule(3).getDriveMotor().getDeviceTemp().setUpdateFrequency(4);
        drivetrain.getModule(3).getSteerMotor().getDeviceTemp().setUpdateFrequency(4);

        
        // Connect the controllers before binding
        if (Constants.OperatorConstants.XBOX_DRIVE) {
            driverXbox = new CommandXboxController(0);
            buttonsXbox = new CommandXboxController(2);
            rightJoystick = null; // Not used in Xbox mode
            leftJoystick = null; // Not used in Xbox mode
        } else {
            rightJoystick = new Joystick(0);
            leftJoystick = new Joystick(1);
            driverXbox = null; // Not used in joystick mode
        }
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        
    // SmartDashboard defaults for data collection and tuning
    SmartDashboard.putBoolean("Record Data", false);
    SmartDashboard.putBoolean("Record Time Data", false);
    SmartDashboard.putNumber("Test Angle", Constants.SetPointConstants.TEST);
    SmartDashboard.putNumber("dist", 0.0);
    SmartDashboard.putNumber("Shooting V", 0.0);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
            OperatorConstants.XBOX_DRIVE ? drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(driverXbox.getLeftY(), 2), 
                            -1, 
                            1
                        ) * Math.signum(driverXbox.getLeftY()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed) 
                    .withVelocityY(MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(driverXbox.getLeftX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(driverXbox.getLeftX()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed) 
                    .withRotationalRate(-MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(driverXbox.getRightX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(driverXbox.getRightX()), 
                        OperatorConstants.ROTATION_DEADBAND) * MaxAngularRate) 
            ) : drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(leftJoystick.getY(), 2), 
                            -1, 
                            1
                        ) * Math.signum(leftJoystick.getY()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed) 
                    .withVelocityY(-MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(leftJoystick.getX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(leftJoystick.getX()), 
                        OperatorConstants.TRANSLATION_DEADBAND) * MaxSpeed) 
                    .withRotationalRate(-MathUtil.applyDeadband(
                        MathUtil.clamp(
                            Math.pow(rightJoystick.getX(), 2), 
                            -1, 
                            1
                        ) * Math.signum(rightJoystick.getX()), 
                        OperatorConstants.ROTATION_DEADBAND) * MaxAngularRate) 
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        if (OperatorConstants.XBOX_DRIVE) {
            driverXbox.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
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
            intakeOnlyCommand 
            /*spindexOnlyCommandIntake*/ )
        );
        buttonsXbox.leftBumper().onFalse(new ParallelCommandGroup(
            intakeRampDown 
            /*spindexRampDownIntake*/)
        );

        driverXbox.leftBumper().whileTrue(outtakeOnlyCommand);
        driverXbox.leftBumper().whileFalse(outtakeRampDown);


        // DPad Right - Hood Angle Up
        buttonsXbox.pov(90).whileTrue(deployIntakeDown);
        // DPad Left - Hood Angle Down
        buttonsXbox.pov(270).whileTrue(deployIntake);

        // DPad Up - Unjam
        buttonsXbox.pov(180).whileTrue(new FeedRollOnly(feedRoller, -0.5).alongWith(new SpindexOnlyCommand(spindexer, -0.5)));

        // DPad Up - Rotate to Angle
        //buttonsXbox.pov(0).whileTrue(autoRotate);
        // DPad Doown - Rotate to Alliance Wall
        //buttonsXbox.pov(180).whileTrue(rotateToAllianceWall);

        // A Button - Far Pass
        buttonsXbox.a().whileTrue(new ParallelCommandGroup(farPassCommand, new WaitCommand(0.5).andThen(new SpindexOnlyCommand(spindexer, 0.5)), new WaitCommand(0.5).andThen(new FeedRollOnly(feedRoller, 0.5)))); 
        buttonsXbox.a().onFalse(shootingRampDown);
        //buttonsXbox.a().whileTrue(new SpindexOnlyCommand(spindexer, 0.5));
        
        // B Button - Mid Pass
        buttonsXbox.b().whileTrue(new ParallelCommandGroup(midPassCommand, new WaitCommand(0.5).andThen(new SpindexOnlyCommand(spindexer, 0.5)), new WaitCommand(0.5).andThen(new FeedRollOnly(feedRoller, 0.5)))); 
        buttonsXbox.b().onFalse(shootingRampDown);
        
        // Y Button - Far Shot
        buttonsXbox.y().whileTrue(farShotCommand); 
        buttonsXbox.y().onFalse(shootingRampDown);
        
        // X Button - Mid Shot
        buttonsXbox.x().whileTrue(midShotCommand);
        buttonsXbox.x().onFalse(shootingRampDown);
    
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}