package org.livoniawarriors.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public  interface ISwerveDriveIo extends Subsystem {
    void updateInputs();
    Translation2d[] getCornerLocations();
    String[] getModuleNames();

    void setTurnMotorBrakeMode(boolean brakeOn);
    void setDriveMotorBrakeMode(boolean brakeOn);
    
    /**
     * Returns the CANcoder absolute angle of the swerve corner in degrees
     * @param wheel Which corner to look at
     * @return The absolute angle of the swerve corner in degrees
     */
    double getCornerAbsAngle(int wheel);
    /**
     * Returns the turn motor absolute angle of the swerve corner in degrees
     * @param wheel Which corner to look at
     * @return The absolute angle of the swerve corner in degrees
     */
    double getCornerAngle(int wheel);
    /**
     * Returns the speed of the swerve corner in meters per second
     * @param wheel Which corner to look at
     * @return The speed of the swerve corner in meters per second
     */
    double getCornerSpeed(int wheel);

    /**
     * Returns the distance the swerve corner as traveled in meters
     * @param wheel Which corner to look at
     * @return The distance the swerve corner as traveled in meters
     */
    double getCornerDistance(int wheel);

    void setCorrectedAngle(int wheel, double angle);

    void setCornerState(int wheel, SwerveModuleState swerveModuleState);

    void SwerveDrive(double xSpeed, double ySpeed, double omega);
    void SwerveDrive(double xSpeed, double ySpeed, double omega, boolean fieldOriented);

    void resetFieldOriented();

    double getMaxDriverSpeed();
    double getMaxDriverOmega();
    double getMinSpeed();
    SwerveModulePosition[] getSwervePositions();
    void setWheelCommand(SwerveModuleState[] states);
    SwerveDriveKinematics getKinematics();
    SwerveModuleState[] getSwerveStates();
}
