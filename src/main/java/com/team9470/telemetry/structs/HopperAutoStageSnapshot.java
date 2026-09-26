package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record HopperAutoStageSnapshot(
        boolean enabled,
        boolean currentCommandIdle,
        boolean topBeamBreakBlocked,
        boolean readyToProbe,
        boolean commandActive,
        boolean lastProbePassed,
        boolean lastProbeTriggeredStage,
        int phaseCode,
        int reasonCode,
        double intakeLoadScoreSec,
        double intakeRollerCurrentAmps,
        double timeSinceIntakeLoadSec,
        double probeAverageCurrentAmps,
        double probeAverageVelocityRps) implements StructSerializable {
    public static final Struct<HopperAutoStageSnapshot> struct = StructGenerator.genRecord(HopperAutoStageSnapshot.class);
}
