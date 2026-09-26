package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record VisionSnapshot(
        int heartbeat,
        boolean fullyConnected,
        boolean visionDisabled,
        int connectedCameraCount,
        int cameraCount,
        int validationStatusCode) implements StructSerializable {
    public static final Struct<VisionSnapshot> struct = StructGenerator.genRecord(VisionSnapshot.class);
}
