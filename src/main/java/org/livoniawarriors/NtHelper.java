package org.livoniawarriors;

import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.FloatArrayEntry;
import edu.wpi.first.networktables.FloatEntry;
import edu.wpi.first.networktables.IntegerArrayEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayEntry;
import edu.wpi.first.networktables.StringEntry;

public class NtHelper {
    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static DoubleEntry getEntry(String key, double backup, boolean persist) {
        var fixKey = fixKey(key);
        DoubleEntry entry = NetworkTableInstance.getDefault().
            getDoubleTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        var value = entry.get();
        return entry;
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static BooleanEntry getEntry(String key, boolean backup, boolean persist) {
        var fixKey = fixKey(key);
        BooleanEntry entry = NetworkTableInstance.getDefault().
            getBooleanTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }
        
    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static FloatEntry getEntry(String key, float backup, boolean persist) {
        var fixKey = fixKey(key);
        FloatEntry entry = NetworkTableInstance.getDefault().
            getFloatTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static IntegerEntry getEntry(String key, long backup, boolean persist) {
        var fixKey = fixKey(key);
        IntegerEntry entry = NetworkTableInstance.getDefault().
            getIntegerTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }
        
    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static StringEntry getEntry(String key, String backup, boolean persist) {
        var fixKey = fixKey(key);
        StringEntry entry = NetworkTableInstance.getDefault().
            getStringTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static DoubleArrayEntry getEntry(String key, double[] backup, boolean persist) {
        var fixKey = fixKey(key);
        DoubleArrayEntry entry = NetworkTableInstance.getDefault().
            getDoubleArrayTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }
        
    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static BooleanArrayEntry getEntry(String key, boolean[] backup, boolean persist) {
        var fixKey = fixKey(key);
        BooleanArrayEntry entry = NetworkTableInstance.getDefault().
            getBooleanArrayTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static FloatArrayEntry getEntry(String key, float[] backup, boolean persist) {
        var fixKey = fixKey(key);
        FloatArrayEntry entry = NetworkTableInstance.getDefault().
            getFloatArrayTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }
        
    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static IntegerArrayEntry getEntry(String key, long[] backup, boolean persist) {
        var fixKey = fixKey(key);
        IntegerArrayEntry entry = NetworkTableInstance.getDefault().
            getIntegerArrayTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @param persist If the value should be remembered
     * @return The entry to get or set values
     */
    public static StringArrayEntry getEntry(String key, String[] backup, boolean persist) {
        var fixKey = fixKey(key);
        StringArrayEntry entry = NetworkTableInstance.getDefault().
            getStringArrayTopic(fixKey).getEntry(backup, new PubSubOption[0]);

        entry.getTopic().setPersistent(persist);
        if(!entry.exists()) {
            entry.set(backup);
        }
        return entry;
    }
        
    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static DoubleEntry getEntry(String key, double backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static DoubleEntry getSetting(String key, double backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static BooleanEntry getEntry(String key, boolean backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static BooleanEntry getSetting(String key, boolean backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static FloatEntry getEntry(String key, float backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static FloatEntry getSetting(String key, float backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static IntegerEntry getEntry(String key, long backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static IntegerEntry getSetting(String key, long backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static StringEntry getEntry(String key, String backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static StringEntry getSetting(String key, String backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static DoubleArrayEntry getEntry(String key, double[] backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static DoubleArrayEntry getSetting(String key, double[] backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static BooleanArrayEntry getEntry(String key, boolean[] backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static BooleanArrayEntry getSetting(String key, boolean[] backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static FloatArrayEntry getEntry(String key, float[] backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static FloatArrayEntry getSetting(String key, float[] backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static IntegerArrayEntry getEntry(String key, long[] backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static IntegerArrayEntry getSetting(String key, long[] backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set values
     */
    public static StringArrayEntry getEntry(String key, String[] backup) {
        return getEntry(key, backup, false);
    }

    /**
     * This creates a NT Entry so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table so all the settings are together.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The entry to get or set the setting
     */
    public static StringArrayEntry getSetting(String key, String[] backup) {
        return getEntry("/Preferences/" + key, backup, true);
    }

    /**
     * This creates a NT Double Entry so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @return The entry to get or set values
     */
    public static DoubleEntry getEntry(String key) {
        //NOTE: This only creates a double entry because Java only allows one function per parameter set, 
        //we can't overload based on return type only...
        return getEntry(key, 0., false);
    }

    private static String fixKey(String key) {
        String newKey;

        if (key.startsWith("/", 0)) {
            newKey = key;
        } else {
            newKey = "/" + key;
        }

        newKey = newKey.replaceAll("//", "/");
        return newKey;
    }
}
