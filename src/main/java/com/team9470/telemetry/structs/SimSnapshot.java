package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record SimSnapshot(
        double flywheelRps,
        double hoodAngleRad,
        int projectileCount,
        double measuredBps) implements StructSerializable {
    public static final Struct<SimSnapshot> struct = StructGenerator.genRecord(SimSnapshot.class);
}
