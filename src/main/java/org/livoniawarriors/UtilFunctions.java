package org.livoniawarriors;

import java.lang.reflect.Field;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

public class UtilFunctions {
    /**
     * This function takes a joystick input from -1 to 1 and removes the center of the stick.
     * This is because Xbox joysticks have awful centering
     * @param input Joystick input to manipulate
     * @param deadband How much of the center we need to remove (Xbox 360 controllers was around 0.2, Xbox One 0.13)
     * @return A value between -1 to 1 that will not drift with stick drift
     */
    public static double deadband(double input, double deadband) {
        double abs = Math.abs(input);

        if (abs > deadband) {
            return Math.signum(input) * ((abs-deadband)/(1-deadband));
        } else {
            return 0;
        }
    }

    /**
     * This function takes a input in degrees and makes it -180 to 180*.
     * If you are in radians, use MathUtil.angleModulus() from WpiLib
     * @param degAngle Angle to reduce
     * @return A value between -180 to 180*
     */
    public static double degreeMod(double degAngle) {
        return MathUtil.inputModulus(degAngle,-180,180);
    }

    /**
     * This uses the Preferences API to save settings over power cycles.
     * This is different in that you don't have to set the default value, it will set it for you.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The value in NetworkTables if it exists, the backup if missing
     */
    public static double getSetting(String key, double backup) {
        if(Preferences.containsKey(key)) {
            //key exists, return the value
            return Preferences.getDouble(key, backup);
        } else {
            //key missing, set default
            Preferences.initDouble(key, backup);
            return backup;
        }
    }

    /**
     * This uses the Preferences API to save settings over power cycles.
     * This is different in that you don't have to set the default value, it will set it for you.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The value in NetworkTables if it exists, the backup if missing
     */
    public static boolean getSetting(String key, boolean backup) {
        if(Preferences.containsKey(key)) {
            //key exists, return the value
            return Preferences.getBoolean(key, backup);
        } else {
            //key missing, set default
            Preferences.initBoolean(key, backup);
            return backup;
        }
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The value in NetworkTables if it exists, the backup if missing
     */
    public static DoubleSubscriber getSettingSub(String key, double backup) {
        DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic("/Preferences/" + key);
        DoublePublisher pub = topic.publish();
        pub.setDefault(backup);
        DoubleSubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        topic.setPersistent(true);
        return sub;
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The value in NetworkTables if it exists, the backup if missing
     */
    public static BooleanSubscriber getNtSub(String key, boolean backup) {
        BooleanTopic topic = NetworkTableInstance.getDefault().getBooleanTopic(key);
        BooleanPublisher pub = topic.publish();
        pub.setDefault(backup);
        BooleanSubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        return sub;
    }

    /**
     * This function adds a periodic function to the schedule.  This will run after the main loop finishes.
     * @param callback Function to run
     * @param periodSeconds How often to run the function in seconds
     * @param offsetSeconds What offset to run this function at
     * @return
     */
    public static boolean addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds) {
        try {
            Field field = RobotBase.class.getDeclaredField("m_robotCopy");
            field.setAccessible(true);
            TimedRobot returnObject = (TimedRobot)field.get(RobotBase.class);
            returnObject.addPeriodic(callback, periodSeconds, offsetSeconds);
            return true;
        } catch (Exception e) {
            //don't do anything, we just return false that it didn't schedule
        } 
        return false;
    }
}
