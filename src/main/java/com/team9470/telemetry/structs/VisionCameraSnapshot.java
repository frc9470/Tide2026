package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record VisionCameraSnapshot(
        boolean connected,
        int heartbeat,
        int resultCount,
        int tagCount,
        double xyStdDevMeters,
        double measurementTimestampSec) implements StructSerializable {
    public static final Struct<VisionCameraSnapshot> struct = StructGenerator.genRecord(VisionCameraSnapshot.class);
}
