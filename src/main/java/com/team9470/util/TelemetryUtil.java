package com.team9470.util;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

public final class TelemetryUtil {
    private TelemetryUtil() {
    }

    public static DoublePublisher publishDouble(NetworkTable table, String key, String unit) {
        var topic = table.getDoubleTopic(key);
        DoublePublisher publisher = topic.publish();
        setUnits(topic, unit);
        return publisher;
    }

    public static void setUnits(DoubleTopic topic, String unit) {
        String jsonUnit = "\"" + unit + "\"";
        topic.setProperty("unit", jsonUnit);
        topic.setProperty("units", jsonUnit);
    }
}
