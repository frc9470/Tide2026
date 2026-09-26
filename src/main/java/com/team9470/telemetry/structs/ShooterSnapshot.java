package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterSnapshot(
        boolean homing,
        boolean firing,
        boolean atSetpoint,
        boolean hoodAtSetpoint,
        boolean flywheelAtSetpoint,
        int stateCode,
        double hoodGoalRad,
        double hoodSetpointRad,
        double hoodPositionRad,
        double hoodErrorRad,
        double hoodVelocityRadPerSec,
        double hoodStatorCurrentAmps,
        double hoodSupplyCurrentAmps,
        double hoodAppliedVolts,
        double nominalFlywheelTargetRps,
        double flywheelTargetRps,
        double flywheelCurrentRps,
        double flywheelErrorRps,
        double flywheelAppliedVolts,
        double flywheelSupplyCurrentAmps,
        double flywheelStatorCurrentAmps) implements StructSerializable {
    public static final Struct<ShooterSnapshot> struct = StructGenerator.genRecord(ShooterSnapshot.class);
}
