package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record HopperSnapshot(
        boolean running,
        double hopperCommandedVolts,
        double feederCommandedVolts,
        double hopperVelocityRps,
        double hopperSupplyCurrentAmps,
        boolean topBeamBreakRawBlocked,
        boolean topBeamBreakBlocked) implements StructSerializable {
    public static final Struct<HopperSnapshot> struct = StructGenerator.genRecord(HopperSnapshot.class);
}
