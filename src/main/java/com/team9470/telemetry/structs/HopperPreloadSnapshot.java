package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record HopperPreloadSnapshot(
        boolean active,
        int phaseCode,
        int faultCode) implements StructSerializable {
    public static final Struct<HopperPreloadSnapshot> struct = StructGenerator.genRecord(HopperPreloadSnapshot.class);
}
