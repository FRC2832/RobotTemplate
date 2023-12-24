// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.odometry.Odometry;
import org.livoniawarriors.odometry.SimSwerveGyro;
import org.livoniawarriors.swerve.DriveXbox;
import org.livoniawarriors.swerve.SwerveDriveSim;
import org.livoniawarriors.swerve.SwerveDriveTrain;

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
    private XboxController driverController;

    public RobotContainer() {
        driverController = new XboxController(0);

        String serNum = RobotController.getSerialNumber();
        SmartDashboard.putString("Serial Number", serNum);
        //known Rio serial numbers:
        //031b525b = buzz
        //03064db7 = big buzz

        //subsystems used in all robots
        Odometry odometry = new Odometry();

        //build the robot based on the Rio ID of the robot
        if (Robot.isSimulation() || (serNum.equals("031b525b")) || (serNum.equals("03064db7"))) {
            //either buzz or simulation
            swerveDrive = new SwerveDriveTrain(new SwerveDriveSim(), odometry::getGyroRotation);
            odometry.setGyroHardware(new SimSwerveGyro(swerveDrive));
        } else {
            //competition robot
        }
        
        odometry.setSwerveDrive(swerveDrive);
        odometry.setStartingPose(new Pose2d(1.92, 2.79, new Rotation2d(0)));
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
