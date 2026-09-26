package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record SuperstructureSnapshot(
        boolean aligned,
        boolean shotValid,
        boolean sotmFireSafe,
        boolean canFire,
        boolean releaseAllowed,
        double rotationCommandRadPerSec,
        double rotationErrorRad) implements StructSerializable {
    public static final Struct<SuperstructureSnapshot> struct = StructGenerator.genRecord(SuperstructureSnapshot.class);
}
