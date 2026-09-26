package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record YShotSnapshot(
        boolean active,
        double requestedRps,
        double requestedHoodRad,
        double appliedRps,
        double appliedHoodRad,
        double distanceMeters,
        boolean readyToFeed) implements StructSerializable {
    public static final Struct<YShotSnapshot> struct = StructGenerator.genRecord(YShotSnapshot.class);
}
