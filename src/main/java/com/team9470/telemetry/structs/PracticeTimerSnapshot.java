package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record PracticeTimerSnapshot(
        int phaseCode,
        boolean active,
        boolean complete,
        boolean practiceMatchTypeDetected,
        boolean usingDsMatchTime,
        int runId,
        double nowSec,
        double runStartSec,
        double phaseStartSec,
        double phaseElapsedSec,
        double phaseRemainingSec,
        double totalElapsedSec,
        double totalRemainingSec,
        double dsMatchTimeSec,
        int startSourceCode,
        int zoneCode,
        boolean zoneActive,
        boolean zoneKnown,
        boolean endgame,
        double zoneRemainingSec,
        double teleopElapsedSec,
        double teleopRemainingSec) implements StructSerializable {
    public static final Struct<PracticeTimerSnapshot> struct = StructGenerator.genRecord(PracticeTimerSnapshot.class);
}
