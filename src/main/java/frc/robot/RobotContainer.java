// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.leds.TestLeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.swervedrive.SwerveSubsystem;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private SwerveSubsystem swerveDrive;
    private LedSubsystem leds;
    private Vision vision;

    private XboxController driverController;

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        driverController = new XboxController(0);

        String swerveDirectory = "swerve/infinity";
        //subsystems used in all robots
        swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveDirectory));
        leds = new LedSubsystem(0, 10);
        vision = new Vision(swerveDrive);
        vision.addCamera(new AprilTagCamera("left",
            new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
            new Translation3d(Units.inchesToMeters(12.056),
                            Units.inchesToMeters(10.981),
                            Units.inchesToMeters(8.44)),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)));

        vision.addCamera(new AprilTagCamera("right",
            new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
            new Translation3d(Units.inchesToMeters(12.056),
                            Units.inchesToMeters(-10.981),
                            Units.inchesToMeters(8.44)),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)));

        vision.addCamera(new AprilTagCamera("center",
            new Rotation3d(0, Units.degreesToRadians(-18), Math.toRadians(180)),
            new Translation3d(Units.inchesToMeters(-4.628),
                                Units.inchesToMeters(-10.687),
                                Units.inchesToMeters(16.129)),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)));

        //add some buttons to press for development
        /*
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.WheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDrive, MoveWheels.WheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.WheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsDiamond()));
        */
        SmartDashboard.putData("Test Leds", new TestLeds(leds));

        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("flashRed", new LightningFlash(leds, Color.kFirstRed));
        NamedCommands.registerCommand("flashBlue", new LightningFlash(leds, Color.kFirstBlue));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings() {
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = swerveDrive.driveCommand(
            () -> MathUtil.applyDeadband(driverController.getLeftY() * -1, 0.05),
            () -> MathUtil.applyDeadband(driverController.getLeftX() * -1, 0.05),
            () -> driverController.getRightX() * -1);

        //setup default commands that are used for driving
        swerveDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        leds.setDefaultCommand(new RainbowLeds(leds));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
