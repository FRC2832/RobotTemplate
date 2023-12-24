package org.livoniawarriors.odometry;

import org.livoniawarriors.Logger;
import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    private boolean PLOT_SWERVE_CORNERS = false;
    IGyroHardware hardware;
    SwerveDriveOdometry odometry;
    SwerveDriveTrain drive;
    Pose2d robotPose;
    private Field2d field;
    private Translation2d[] swervePositions;

    public Odometry() {
        super();
        hardware = new BlankGyro();
        robotPose = new Pose2d();
        field = new Field2d();

        SmartDashboard.putData("Field", field);
        Logger.RegisterSensor("Gyro Yaw", this::getGyroAngle);
        Logger.RegisterSensor("Gyro Pitch", this::getGyroPitch);
        Logger.RegisterSensor("Gyro Roll", this::getGyroRoll);
        Logger.RegisterSensor("Gyro X Accel", ()->hardware.getXAccel());
        Logger.RegisterSensor("Gyro Y Accel", ()->hardware.getYAccel());
        Logger.RegisterSensor("Gyro Z Accel", ()->hardware.getZAccel());
    }

    public void setSwerveDrive(SwerveDriveTrain drive) {
        this.drive = drive;
        swervePositions = drive.getCornerLocations();
        odometry = new SwerveDriveOdometry(drive.getKinematics(), getGyroRotation(), drive.getSwerveStates());
    }

    public void setGyroHardware(IGyroHardware hardware) {
        this.hardware = hardware;
        hardware.updateHardware();
    }
   
    @Override
    public void periodic() {
        hardware.updateHardware();
        Rotation2d heading = getGyroRotation();
        SwerveModulePosition[] states = drive.getSwerveStates();
        robotPose = odometry.update(heading, states);
        field.setRobotPose(robotPose);

        if(PLOT_SWERVE_CORNERS) {
            // Update the poses for the swerveModules. Note that the order of rotating the
            // position and then adding the translation matters
            var modulePoses = new Pose2d[swervePositions.length];
            for (int i = 0; i < swervePositions.length; i++) {
                Translation2d modulePositionFromChassis = swervePositions[i].rotateBy(heading).plus(robotPose.getTranslation());

                // Module's heading is it's angle relative to the chassis heading
                modulePoses[i] = new Pose2d(modulePositionFromChassis,
                    states[i].angle.plus(robotPose.getRotation()));
            }
            field.getObject("Swerve Modules").setPoses(modulePoses);
        }
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getGyroRotation(), drive.getSwerveStates(), pose);
    }

    public void resetHeading() {
        //reset the robot back to it's spot, just facing forward now
        Pose2d pose = new Pose2d(robotPose.getTranslation(),Rotation2d.fromDegrees(0));
        odometry.resetPosition(getGyroRotation(), drive.getSwerveStates(), pose);
    }

    public Pose2d getPose() {
        return robotPose;
    }

    public Rotation2d getHeading() {
        return robotPose.getRotation();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    public double getGyroAngle() {
        return hardware.getGyroAngle();
    }

    public double getGyroPitch() {
        return hardware.getPitchAngle();
    }

    public double getGyroRoll() {
        return hardware.getRollAngle();
    }
}
