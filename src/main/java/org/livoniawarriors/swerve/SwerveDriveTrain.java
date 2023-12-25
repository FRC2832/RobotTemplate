package org.livoniawarriors.swerve;

import java.util.function.Supplier;

import org.livoniawarriors.UtilFunctions;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
    /** The fastest rate we want the drive wheels to change speeds in m/s */
    final String MAX_ACCEL_KEY = "Swerve Drive/Max Wheel Accel";
    /** The fastest rate we want the swerve wheels to turn in deg/s */
    final String MAX_OMEGA_KEY = "Swerve Drive/Max Wheel Omega";
    /** The max speed possible with the swerve wheels in m/s */
    final String MIN_SPEED_KEY = "Swerve Drive/Min Speed";
    final String MAX_SPEED_KEY = "Swerve Drive/Max Speed";
    /** The angle in degrees we want the swerve to invert the request to get to position faster */
    final String OPTIMIZE_ANGLE_KEY = "Swerve Drive/Optimize Angle";
    final String FIELD_ORIENTED = "Swerve Drive/Field Oriented";

    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModulePosition[] swerveStates;
    private SwerveModuleState[] swerveTargets;
    private double gyroOffset = 0;
    private PIDController pidZero = new PIDController(0.15, 0.001, 0);
    private PIDController[] turnPid;
    private SwerveModuleState[] currentState;
    private boolean optimize;
    private boolean resetZeroPid;
    private double minSpeed;
    private double maxSpeed;
    private Rotation2d currentHeading;
    private Rotation2d fieldOffset;
    private Supplier<Rotation2d> gyroSupplier;
    private boolean lastTeleop;

    //input settings
    private DoubleSubscriber driverMaxSpeed;
    private DoubleSubscriber driverMaxOmega;
    private DoubleSubscriber[] wheelOffsetSetting;

    //output data
    private DoublePublisher[] wheelCalcAngle;
    private DoublePublisher[] wheelCommandAngle;
    private DoublePublisher[] wheelRequestAngle;
    private DoublePublisher[] wheelCommandSpeed;
    private DoublePublisher[] wheelRequestSpeed;
    private DoublePublisher swerveXSpeed;
    private DoublePublisher swerveYSpeed;
    private DoublePublisher swerveOmega;
    private DoubleArrayPublisher swerveStatePub;
    private DoubleArrayPublisher swerveRequestPub;

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo, Supplier<Rotation2d> gyroSupplier) {
        super();
        this.hardware = hSwerveDriveIo;
        this.gyroSupplier = gyroSupplier;
        optimize = true;
        resetZeroPid = false;
        int numWheels = hardware.getCornerLocations().length;
        fieldOffset = new Rotation2d();

        //initialize module names
        String[] moduleNames = hardware.getModuleNames();

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(hSwerveDriveIo.getCornerLocations());
        
        //initialize the swerve states
        swerveStates = new SwerveModulePosition[numWheels];
        swerveTargets = new SwerveModuleState[numWheels];
        turnPid = new PIDController[numWheels];
        currentState = new SwerveModuleState[numWheels];
        wheelOffsetSetting = new DoubleSubscriber[numWheels];
        wheelCalcAngle = new DoublePublisher[numWheels];
        wheelCommandAngle = new DoublePublisher[numWheels];
        wheelRequestAngle = new DoublePublisher[numWheels];
        wheelCommandSpeed = new DoublePublisher[numWheels];
        wheelRequestSpeed = new DoublePublisher[numWheels];
        for(int wheel = 0; wheel < numWheels; wheel++) {
            swerveStates[wheel] = new SwerveModulePosition();
            swerveTargets[wheel] = new SwerveModuleState();
            turnPid[wheel] = new PIDController(5,1.8,0);
            currentState[wheel] = new SwerveModuleState();
            wheelOffsetSetting[wheel] = UtilFunctions.getSettingSub("/Swerve Drive/Wheel Offset " + moduleNames[wheel] + "_deg", 0);
            wheelCalcAngle[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Calc Angle_deg", 0);
            wheelCommandAngle[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Command Angle_deg", 0);
            wheelRequestAngle[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Request Angle_deg", 0);
            wheelCommandSpeed[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Command Speed_mps", 0);
            wheelRequestSpeed[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Request Speed_mps", 0);
        }

        /** How fast we want the driver to go during normal operation in m/s */
        driverMaxSpeed = UtilFunctions.getSettingSub("/Swerve Drive/Max Driver Speed_mps", 3);
        /** How fast we want the driver to turn during normal operation in deg/s */
        driverMaxOmega = UtilFunctions.getSettingSub("/Swerve Drive/Max Driver Omega_dps", 625);   //1.8 * Pi rad/sec

        swerveXSpeed = UtilFunctions.getNtPub("/Swerve Drive/X Speed_mps", 0);
        swerveYSpeed = UtilFunctions.getNtPub("/Swerve Drive/Y Speed_mps", 0);
        swerveOmega = UtilFunctions.getNtPub("/Swerve Drive/Omega_dps", 0);
        swerveStatePub = UtilFunctions.getNtPub("/Swerve Drive/Module States", new double[0]);
        swerveRequestPub = UtilFunctions.getNtPub("/Swerve Drive/Module Requests", new double[0]);
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();
        currentHeading = gyroSupplier.get();

        //read the swerve corner state
        for(int wheel = 0; wheel < swerveStates.length; wheel++) {
            double offset = wheelOffsetSetting[wheel].get(0);
            double angle = hardware.getCornerAbsAngle(wheel) - offset;
            swerveStates[wheel].angle = Rotation2d.fromDegrees(angle);
            swerveStates[wheel].distanceMeters = hardware.getCornerDistance(wheel);

            currentState[wheel].angle = swerveStates[wheel].angle;
            currentState[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);

            wheelCalcAngle[wheel].set(swerveStates[wheel].angle.getDegrees());
        }

        //when we are disabled, reset the turn pids as we don't want to act on the "error" when reenabled
        boolean curTeleop = DriverStation.isTeleopEnabled();
        if(lastTeleop == false && curTeleop == true || resetZeroPid) {
            for (var pid : turnPid) {
                pid.reset();
            }
            gyroOffset = currentHeading.getDegrees();
            fieldOffset = currentHeading;
        }
        lastTeleop = curTeleop;
        resetZeroPid = false;

        PushSwerveStates(currentState,swerveTargets);
        minSpeed = UtilFunctions.getSetting(MIN_SPEED_KEY, 0.5);
        maxSpeed = UtilFunctions.getSetting(MAX_SPEED_KEY, 5);
    }

    public void SwerveDrive(double xSpeed, double ySpeed, double turn) {
        SwerveDrive(xSpeed, ySpeed, turn, UtilFunctions.getSetting(FIELD_ORIENTED, true));
    }

    public void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;

        if (Math.abs(turn) > 0.1) {
            //if a turn is requested, reset the zero for the drivetrain
            gyroOffset = currentHeading.getDegrees();
            pidZero.reset();
        } else {
            //straighten the robot
            turn = pidZero.calculate(currentHeading.getDegrees(),gyroOffset);
        }

        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, currentHeading.minus(fieldOffset));
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        }
        
        //log the request
        swerveXSpeed.set(xSpeed);
        swerveYSpeed.set(ySpeed);
        swerveOmega.set(Math.toDegrees(turn));

        //calculate the states from the speeds
        SwerveModuleState[] requestStates = kinematics.toSwerveModuleStates(speeds);
        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(requestStates, maxSpeed);

        for(int i=0; i<requestStates.length; i++) {
            wheelRequestAngle[i].set(requestStates[i].angle.getDegrees());
            wheelRequestSpeed[i].set(requestStates[i].speedMetersPerSecond);
        }

        //filter the swerve wheels
        if(optimize) {
            requestStates = optimizeSwerve(requestStates, currentState, true);
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

            wheelCommandAngle[i].set(requestStates[i].angle.getDegrees());
            wheelCommandSpeed[i].set(requestStates[i].speedMetersPerSecond);
        }
        swerveTargets = requestStates;
    }

    public void setWheelCommand(SwerveModuleState[] requestStates) {
        resetZeroPid = true;
        swerveTargets = requestStates;
        for(int i=0; i<requestStates.length; i++) {
            if(optimize) {
                requestStates = optimizeSwerve(requestStates, currentState, false);
            }
                
            var volts = -turnPid[i].calculate(swerveStates[i].angle.getRadians(),requestStates[i].angle.getRadians());
            hardware.setDriveCommand(i, ControlMode.Velocity, requestStates[i].speedMetersPerSecond);
            hardware.setTurnCommand(i, ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());

            wheelRequestAngle[i].set(requestStates[i].angle.getDegrees());
            wheelRequestSpeed[i].set(requestStates[i].speedMetersPerSecond);
        }

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(requestStates);
        //log the request
        swerveXSpeed.set(speeds.vxMetersPerSecond);
        swerveYSpeed.set(speeds.vyMetersPerSecond);
        swerveOmega.set(Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    public SwerveModuleState[] optimizeSwerve(SwerveModuleState[] requestStates, SwerveModuleState[] currentState, boolean stopTurnAtZero) {
        SwerveModuleState[] outputStates = new SwerveModuleState[requestStates.length];
        //we use a little larger optimize angle since drivers turning 90* is a pretty common operation
        double optimizeAngle = UtilFunctions.getSetting(OPTIMIZE_ANGLE_KEY, 120);
        double maxAccel = UtilFunctions.getSetting(MAX_ACCEL_KEY, 42);
        double maxOmega = UtilFunctions.getSetting(MAX_OMEGA_KEY, 3000);

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            outputStates[i] = new SwerveModuleState();

            //figure out if we should invert the request
            double angleReq = requestStates[i].angle.getDegrees();
            double curAngle = currentState[i].angle.getDegrees();
            double speedReq = requestStates[i].speedMetersPerSecond;
            double deltaMod = MathUtil.inputModulus(angleReq - curAngle,-180,180);
            if(Math.abs(deltaMod) > optimizeAngle) {
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
            angleReq = MathUtil.inputModulus(angleReq, curAngle - 180, curAngle + 180);
            double delta = angleReq - curAngle;
            if(delta > maxAngleDelta) {
                angleReq = curAngle + maxAngleDelta;
            } else if (delta < -maxAngleDelta) {
                angleReq = curAngle - maxAngleDelta;
            } else {
                //angle request if fine
            }
            //make it back into a Rotation2d
            outputStates[i].angle = Rotation2d.fromDegrees(angleReq);

            //check to see if the robot request is moving
            if (stopTurnAtZero && Math.abs(speedReq) < minSpeed) {
                //stop the requests if there is no movement
                outputStates[i].angle = currentState[i].angle;
                //take out minimal speed so that the motors don't jitter
                outputStates[i].speedMetersPerSecond = 0;
            }
        }
        return outputStates;
    }

    private void PushSwerveStates(SwerveModuleState[] state, SwerveModuleState[] request) {
        int size = state.length;
        double[] states = new double[size * 2];
        double[] requests = new double[size * 2];
        for(int i=0; i<size; i++) {
            states[(i * 2)] = state[i].angle.getDegrees();
            states[(i * 2)+1] = state[i].speedMetersPerSecond;
            requests[(i * 2)] = request[i].angle.getDegrees();
            requests[(i * 2)+1] = request[i].speedMetersPerSecond;
        }
        swerveStatePub.set(states);
        swerveRequestPub.set(requests);
    }

    public void stopWheels() {
        SwerveDrive(0,0,0);
    }
    public void resetFieldOriented() {
        fieldOffset = currentHeading;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
   
    public SwerveModulePosition[] getSwervePositions() {
        return swerveStates;
    }

    public SwerveModuleState[] getSwerveStates() {
        return currentState;
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

    public void setOptimizeOn(boolean enabled) {
        optimize = enabled;
    }
    
    public boolean getOptimizeOn() {
        return optimize;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxDriverSpeed() {
        return driverMaxSpeed.get();
    }

    public double getMaxDriverOmega() {
        return Math.toRadians(driverMaxOmega.get());
    }

    public double getMinSpeed() {
        return minSpeed;
    }
}
