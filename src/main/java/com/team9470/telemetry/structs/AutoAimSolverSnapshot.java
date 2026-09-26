package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record AutoAimSolverSnapshot(
        int modeCode,
        boolean feedModeActive,
        double robotXBlueMeters,
        double distanceMeters,
        double distanceNoLookaheadMeters,
        double naiveAirTimeSec,
        boolean valid,
        double targetYawRad,
        double targetOmegaRadPerSec,
        double hoodCommandRad,
        double flywheelRps,
        double shooterFieldSpeedMps,
        double shooterFieldAccelMps2) implements StructSerializable {
    public static final Struct<AutoAimSolverSnapshot> struct = StructGenerator.genRecord(AutoAimSolverSnapshot.class);
}
