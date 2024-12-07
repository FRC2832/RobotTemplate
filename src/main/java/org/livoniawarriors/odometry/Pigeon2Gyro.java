package org.livoniawarriors.odometry;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;

public class Pigeon2Gyro implements IGyroHardware {
    private Pigeon2 pigeon;

    StatusSignal<LinearAcceleration> accelX;
    StatusSignal<LinearAcceleration> accelY;
    StatusSignal<LinearAcceleration> accelZ;
    StatusSignal<Angle> yaw;
    StatusSignal<Angle> pitch;
    StatusSignal<Angle> roll;
    
    List<StatusSignal<?>> allSignals;

    public Pigeon2Gyro(int id) {
        this(id, null);
    }

    public Pigeon2Gyro(int id, String bus_name) {
        if(bus_name != null) {
            pigeon = new Pigeon2(id, bus_name);
        } else {
            pigeon = new Pigeon2(id);
        }

        //get all our signal grabbers
        accelX = pigeon.getAccelerationX();
        accelY = pigeon.getAccelerationY();
        accelZ = pigeon.getAccelerationZ();
        yaw = pigeon.getYaw();
        pitch = pigeon.getPitch();
        roll = pigeon.getRoll();
        allSignals = Arrays.asList(accelX, accelY, accelZ, yaw, pitch, roll);

        //set the optimum rates for our signals
        for (StatusSignal<?> signal : allSignals) {
            //we run at 50Hz (20ms loops), so no need to grab the data faster
            signal.setUpdateFrequency(50);
        }
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateHardware() {
        //no need to update anymore, as the StatusSignals will auto update and cache values
    }

    @Override
    public double getGyroAngle() {
        return yaw.getValueAsDouble();
    }

    @Override
    public double getPitchAngle() {
        return pitch.getValueAsDouble();
    }

    @Override
    public double getRollAngle() {
        return roll.getValueAsDouble();
    }

    @Override
    public double getXAccel() {
        return accelX.getValueAsDouble();
    }

    @Override
    public double getYAccel() {
        return accelY.getValueAsDouble();
    }

    @Override
    public double getZAccel() {
        return accelZ.getValueAsDouble();
    }
}
