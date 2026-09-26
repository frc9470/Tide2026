package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record DriveStatusSnapshot(
        double timestampSec,
        double odometryFrequencyHz,
        double odometryPeriodSec) implements StructSerializable {
    public static final Struct<DriveStatusSnapshot> struct = StructGenerator.genRecord(DriveStatusSnapshot.class);
}
