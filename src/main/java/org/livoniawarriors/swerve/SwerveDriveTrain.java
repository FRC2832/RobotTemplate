package org.livoniawarriors.swerve;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
    /** The fastest rate we want the drive wheels to change speeds in m/s */
    final String MAX_ACCEL_KEY = "Swerve Drive/Max Wheel Accel";
    /** The fastest rate we want the swerve wheels to turn in deg/s */
    final String MAX_OMEGA_KEY = "Swerve Drive/Max Wheel Omega";
    /** How fast we want the driver to go during normal operation in m/s */
    final String MAX_DRIVER_SPEED_KEY = "Swerve Drive/Max Driver Speed";
    /** How fast we want the driver to turn during normal operation in deg/s */
    final String MAX_DRIVER_OMEGA_KEY = "Swerve Drive/Max Driver Omega";
    /** The max speed possible with the swerve wheels in m/s */
    final String MIN_SPEED_KEY = "Swerve Drive/Min Speed";
    final String MAX_SPEED_KEY = "Swerve Drive/Max Speed";
    /** How slow we want the driver to go in turtle mode */
    final String TURTLE_SPEED_KEY = "Swerve Drive/Turtle Speed";
    /** The angle in degrees we want the swerve to invert the request to get to position faster */
    final String OPTOMIZE_ANGLE_KEY = "Swerve Drive/Optomize Angle";

    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModulePosition[] swerveStates;
    private SwerveModuleState[] swerveTargets;
    private Pose2d robotPose;
    private String[] moduleNames;
    private double gyroOffset = 0;
    private PIDController pidZero = new PIDController(0.15, 0.001, 0);
    private PIDController[] drivePid;
    private PIDController[] turnPid;
    private SwerveModuleState[] currentState;
    private boolean optimize;
    private double minSpeed;
    private double maxSpeed;

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo) {
        register();
        this.hardware = hSwerveDriveIo;
        optimize = true;

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(hSwerveDriveIo.getCornerLocations());
        
        //initialize the swerve states
        swerveStates = new SwerveModulePosition[hardware.GetNumWheels()];
        swerveTargets = new SwerveModuleState[hardware.GetNumWheels()];
        drivePid = new PIDController[hardware.GetNumWheels()];
        turnPid = new PIDController[hardware.GetNumWheels()];
        currentState = new SwerveModuleState[hardware.GetNumWheels()];
        for(int wheel = 0; wheel < hardware.GetNumWheels(); wheel++) {
            swerveStates[wheel] = new SwerveModulePosition();
            swerveTargets[wheel] = new SwerveModuleState();
            drivePid[wheel] = new PIDController(0.5, 0.03, 0.0);
            turnPid[wheel] = new PIDController(5,1.8,0);
            currentState[wheel] = new SwerveModuleState();
        }

        //initialize module names
        moduleNames = new String[hardware.GetNumWheels()];
        if(hardware.GetNumWheels() == 4) {
            moduleNames[0] = "Swerve FL/";
            moduleNames[1] = "Swerve FR/";
            moduleNames[2] = "Swerve RL/";
            moduleNames[3] = "Swerve RR/";
        } else {
            for(int i=0; i<hardware.GetNumWheels(); i++) {
                moduleNames[i] = "Swerve "+ i + "/";
            }
        }
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();

        //read the swerve corner state
        for(int wheel = 0; wheel < hardware.GetNumWheels(); wheel++) {
            double offset = UtilFunctions.getSetting(moduleNames[wheel] + "Wheel Offset", 0);
            double angle = hardware.getCornerAbsAngle(wheel) - offset;
            swerveStates[wheel].angle = Rotation2d.fromDegrees(angle);
            swerveStates[wheel].distanceMeters = hardware.getCornerDistance(wheel);

            currentState[wheel].angle = swerveStates[wheel].angle;
            currentState[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);

            if(DriverStation.isDisabled() || DriverStation.isAutonomous()) {
                //when we are disabled, reset the turn pids as we don't want to act on the "error" when reenabled
                turnPid[wheel].reset();
                gyroOffset = 0;// getHeading().getDegrees();
            }

            SmartDashboard.putNumber(moduleNames[wheel] + "Calc Angle", swerveStates[wheel].angle.getDegrees());
        }

        Logger.PushSwerveStates(currentState,swerveTargets);
        minSpeed = UtilFunctions.getSetting(MIN_SPEED_KEY, 0.5);
        maxSpeed = UtilFunctions.getSetting(MAX_SPEED_KEY, 5);
    }

    public void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;

        double currentHeading = 0; //getHeading().getDegrees();
        if (Math.abs(turn) > 0.1) {
            //if a turn is requested, reset the zero for the drivetrain
            gyroOffset = currentHeading;
            pidZero.reset();
        } else {
            //straighten the robot
            turn = pidZero.calculate(currentHeading,gyroOffset);
        }

        if (fieldOriented) {
            var angle = robotPose.getRotation();
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, angle);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        }
        
        SmartDashboard.putNumber("Swerve XSpeed", xSpeed);
        SmartDashboard.putNumber("Swerve YSpeed", ySpeed);
        SmartDashboard.putNumber("Swerve Turn", turn);

        //calculate the states from the speeds
        SwerveModuleState[] requestStates = kinematics.toSwerveModuleStates(speeds);
        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(requestStates, maxSpeed);

        for(int i=0; i<requestStates.length; i++) {
            SmartDashboard.putNumber(moduleNames[i] + "Requested Angle", requestStates[i].angle.getDegrees());
            SmartDashboard.putNumber(moduleNames[i] + "Requested Speed", requestStates[i].speedMetersPerSecond);
        }

        //filter the swerve wheels
        if(optimize) {
            requestStates = optomizeSwerve(requestStates, currentState);
        }

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            //turn software PID
            if (Math.abs(swerveStates[i].angle.minus(requestStates[i].angle).getDegrees()) < 1) {
                //reset the PID to remove all the I term error so we don't overshoot and rebound
                turnPid[i].reset();
            }
            var turnVolts = -turnPid[i].calculate(swerveStates[i].angle.getRadians(), requestStates[i].angle.getRadians());
            hardware.setTurnCommand(i, ControlMode.PercentOutput, turnVolts / RobotController.getBatteryVoltage());

            // velocity drive mode
            hardware.setDriveCommand(i, ControlMode.Velocity, requestStates[i].speedMetersPerSecond);

            SmartDashboard.putNumber(moduleNames[i] + "Command Angle", requestStates[i].angle.getDegrees());
            SmartDashboard.putNumber(moduleNames[i] + "Command Speed", requestStates[i].speedMetersPerSecond);
        }
        swerveTargets = requestStates;
    }

    public void setWheelCommand(SwerveModuleState[] requestStates) {
        swerveTargets = requestStates;
        for(int i=0; i<requestStates.length; i++) {
            if(optimize) {
                requestStates = optomizeSwerve(requestStates, currentState);
            }
                
            var volts = -turnPid[i].calculate(swerveStates[i].angle.getRadians(),requestStates[i].angle.getRadians());
            hardware.setDriveCommand(i, ControlMode.Velocity, requestStates[i].speedMetersPerSecond);
            hardware.setTurnCommand(i, ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
        }
    }

    public SwerveModuleState[] optomizeSwerve(SwerveModuleState[] requestStates, SwerveModuleState[] currentState) {
        SwerveModuleState[] outputStates = new SwerveModuleState[requestStates.length];
        //we use a little larger optomize angle since drivers turning 90* is a pretty common operation
        double optomizeAngle = UtilFunctions.getSetting(OPTOMIZE_ANGLE_KEY, 120);
        double maxAccel = UtilFunctions.getSetting(MAX_ACCEL_KEY, 10);
        double maxOmega = UtilFunctions.getSetting(MAX_OMEGA_KEY, 3000);

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            outputStates[i] = new SwerveModuleState();

            //figure out if we should invert the request
            double angleReq = requestStates[i].angle.getDegrees();
            double curAngle = currentState[i].angle.getDegrees();
            double speedReq = requestStates[i].speedMetersPerSecond;
            double deltaMod = MathUtil.inputModulus(angleReq - curAngle,-180,180);
            if(Math.abs(deltaMod) > optomizeAngle) {
                angleReq = angleReq - 180;
                speedReq = -requestStates[i].speedMetersPerSecond;
            }

            //smooth out drive command
            
            double maxSpeedDelta = maxAccel * TimedRobot.kDefaultPeriod;           //acceleration * loop time
            //whatever value is bigger flips when forwards vs backwards
            double value1 = currentState[i].speedMetersPerSecond - maxSpeedDelta;
            double value2 = currentState[i].speedMetersPerSecond + maxSpeedDelta;
            outputStates[i].speedMetersPerSecond = MathUtil.clamp(
                speedReq,                  //current request
                Math.min(value1, value2),                               //last request minimum
                Math.max(value1, value2));                              //last request maximum

            //smooth out turn command
            double maxAngleDelta = maxOmega * TimedRobot.kDefaultPeriod;           //acceleration * loop time
            if (Math.abs(speedReq) > minSpeed) {
                angleReq = MathUtil.inputModulus(angleReq, curAngle - 180, curAngle + 180);
            } else {
                angleReq = curAngle;
            }
            double delta = angleReq - curAngle;
            if(delta > maxAngleDelta) {
                angleReq = curAngle + maxAngleDelta;
            } else if (delta < -maxAngleDelta) {
                angleReq = curAngle - maxAngleDelta;
            } else {
                //angle request if fine
            }
            outputStates[i].angle = Rotation2d.fromDegrees(angleReq);

            //check to see if the robot request is moving
            if (Math.abs(speedReq) < minSpeed) {
                //stop the requests if there is no movement
                outputStates[i].angle = currentState[i].angle;
                //take out minimal speed so that the motors don't jitter
                outputStates[i].speedMetersPerSecond = 0;
            }
        }
        return outputStates;
    }

    
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
   
    public SwerveModulePosition[] getSwerveStates() {
        return swerveStates;
    }

    public void setPose(Pose2d robotPose) {
        this.robotPose = robotPose;        
    }

    public void setTurnMotorBrakeMode(boolean brakeOn) {
        hardware.setTurnMotorBrakeMode(brakeOn);
    }

    public void setDriveMotorBrakeMode(boolean brakeOn) {
        hardware.setDriveMotorBrakeMode(brakeOn);
    }

    public Translation2d[] getCornerLocations() {
        return hardware.getCornerLocations();
    }

    public void setOptomizeOn(boolean enabled) {
        optimize = enabled;
    }
}
