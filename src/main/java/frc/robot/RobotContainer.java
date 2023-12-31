// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.leds.TestLeds;
import org.livoniawarriors.odometry.Odometry;
import org.livoniawarriors.odometry.Pigeon2Gyro;
import org.livoniawarriors.odometry.SimSwerveGyro;
import org.livoniawarriors.swerve.DriveXbox;
import org.livoniawarriors.swerve.MoveWheels;
import org.livoniawarriors.swerve.SwerveDriveSim;
import org.livoniawarriors.swerve.SwerveDriveTrain;
import org.livoniawarriors.swerve.SwerveHw23;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private SwerveDriveTrain swerveDrive;
    private Odometry odometry;
    private LedSubsystem leds;

    private XboxController driverController;

    public RobotContainer() {
        driverController = new XboxController(0);

        String serNum = RobotController.getSerialNumber();
        SmartDashboard.putString("Serial Number", serNum);
        //known Rio serial numbers:
        //031b525b = buzz
        //03064db7 = big buzz

        //subsystems used in all robots
        odometry = new Odometry();
        leds = new LedSubsystem(0, 10);

        //build the robot based on the Rio ID of the robot
        if (Robot.isSimulation() || (serNum.equals("031b525b")) || (serNum.equals("03064db7"))) {
            //either buzz or simulation
            swerveDrive = new SwerveDriveTrain(new SwerveDriveSim(), odometry);
            odometry.setGyroHardware(new SimSwerveGyro(swerveDrive));
        } else {
            //competition robot
            swerveDrive = new SwerveDriveTrain(new SwerveHw23(), odometry);
            odometry.setGyroHardware(new Pigeon2Gyro(0));
        }
        
        odometry.setSwerveDrive(swerveDrive);
        odometry.setStartingPose(new Pose2d(1.92, 2.79, new Rotation2d(0)));

        //add some buttons to press for development
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.WheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDrive, MoveWheels.WheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.WheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsDiamond()));
        SmartDashboard.putData("Test Leds", new TestLeds(leds));
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
        //setup default commands that are used for driving
        swerveDrive.setDefaultCommand(new DriveXbox(swerveDrive, driverController));
        leds.setDefaultCommand(new RainbowLeds(leds));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new Command() {
            
        };
    }
}
