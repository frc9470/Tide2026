package com.team9470.telemetry;

import com.team9470.FieldConstants;
import com.team9470.Constants;
import com.team9470.telemetry.structs.AutoAimSolverSnapshot;
import com.team9470.telemetry.structs.DriveStatusSnapshot;
import com.team9470.telemetry.structs.HopperAutoStageSnapshot;
import com.team9470.telemetry.structs.HopperPreloadSnapshot;
import com.team9470.telemetry.structs.HopperSnapshot;
import com.team9470.telemetry.structs.IntakeSnapshot;
import com.team9470.telemetry.structs.PracticeTimerSnapshot;
import com.team9470.telemetry.structs.ShotReleaseSnapshot;
import com.team9470.telemetry.structs.ShooterCharacterizationSnapshot;
import com.team9470.telemetry.structs.ShooterSnapshot;
import com.team9470.telemetry.structs.SimSnapshot;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.telemetry.structs.TimedShotSnapshot;
import com.team9470.telemetry.structs.VisionCameraSnapshot;
import com.team9470.telemetry.structs.VisionSnapshot;
import com.team9470.telemetry.structs.YShotSnapshot;
import com.team9470.util.AllianceFlipUtil;
import com.team9470.util.TelemetryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.HashMap;
import java.util.Map;

public final class TelemetryManager {
    public static final int VALIDATION_OK = 0;
    public static final int VALIDATION_OUTSIDE_FIELD = 1;
    public static final int VALIDATION_MAX_VELOCITY = 2;
    public static final int VALIDATION_MAX_CORRECTION = 3;

    private static final TelemetryManager instance = new TelemetryManager();
    private static final double kAutoStartupMeasuredTranslationThresholdMps = 0.05;
    private static final double kAutoStartupMeasuredOmegaThresholdRadPerSec = 0.10;
    private static final double kAutoCenterlineTouchDistanceM = Constants.RobotGeometry.kChoreoSideBumperMeters;

    public static TelemetryManager getInstance() {
        return instance;
    }

    private final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry");

    private final NetworkTable driveTable = telemetryTable.getSubTable("Drive");
    private final StructPublisher<Pose2d> drivePosePublisher = driveTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeedsPublisher = driveTable.getStructTopic("Speeds", ChassisSpeeds.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStatesPublisher = driveTable
            .getStructArrayTopic("Modules/States", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargetsPublisher = driveTable
            .getStructArrayTopic("Modules/Targets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositionsPublisher = driveTable
            .getStructArrayTopic("Modules/Positions", SwerveModulePosition.struct).publish();
    private final DoubleArrayPublisher driveModuleVelocityPublisher = driveTable
            .getDoubleArrayTopic("Modules/DriveVelocityRps").publish();
    private final DoubleArrayPublisher driveModuleStatorCurrentPublisher = driveTable
            .getDoubleArrayTopic("Modules/DriveStatorCurrentAmps").publish();
    private final NetworkTable driveCurrentLimitTable = driveTable.getSubTable("CurrentLimit");
    private final DoublePublisher driveCurrentLimitActivePublisher = TelemetryUtil.publishDouble(
            driveCurrentLimitTable, "ActiveStatorAmps", "A");
    private final DoublePublisher driveCurrentLimitNominalPublisher = TelemetryUtil.publishDouble(
            driveCurrentLimitTable, "NominalStatorAmps", "A");
    private final DoublePublisher driveCurrentLimitTurboPublisher = TelemetryUtil.publishDouble(
            driveCurrentLimitTable, "TurboStatorAmps", "A");
    private final BooleanPublisher driveCurrentLimitTurboEnabledPublisher = driveCurrentLimitTable
            .getBooleanTopic("TurboEnabled").publish();
    private final StructPublisher<DriveStatusSnapshot> driveStatusPublisher = driveTable
            .getStructTopic("Status", DriveStatusSnapshot.struct).publish();
    private final NetworkTable driveAimTable = driveTable.getSubTable("Aim");
    private final StructArrayPublisher<Pose2d> driveAimHeadingLinePublisher = driveAimTable
            .getStructArrayTopic("HeadingLine", Pose2d.struct).publish();
    private final StructArrayPublisher<Pose2d> driveAimHubReferenceLinePublisher = driveAimTable
            .getStructArrayTopic("HubReferenceLine", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> driveAimHubCenterPublisher = driveAimTable
            .getStructTopic("HubCenter", Pose2d.struct).publish();
    private final NetworkTable driveAutoTable = driveTable.getSubTable("Auto");
    private final BooleanPublisher driveAutoActivePublisher = driveAutoTable.getBooleanTopic("Active").publish();
    private final DoublePublisher driveAutoLastSampleTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoTable, "LastSampleTimestampSec", "s");
    private final StructPublisher<Pose2d> driveAutoDesiredPosePublisher = driveAutoTable
            .getStructTopic("DesiredPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> driveAutoMeasuredPosePublisher = driveAutoTable
            .getStructTopic("MeasuredPose", Pose2d.struct).publish();
    private final StructPublisher<Translation2d> driveAutoPoseErrorPublisher = driveAutoTable
            .getStructTopic("PoseError", Translation2d.struct).publish();
    private final DoublePublisher driveAutoPoseErrorNormPublisher = TelemetryUtil.publishDouble(
            driveAutoTable, "PoseErrorNormM", "m");
    private final DoublePublisher driveAutoHeadingErrorPublisher = TelemetryUtil.publishDouble(
            driveAutoTable, "HeadingErrorRad", "rad");
    private final StructPublisher<ChassisSpeeds> driveAutoFeedforwardSpeedsPublisher = driveAutoTable
            .getStructTopic("FeedforwardSpeeds", ChassisSpeeds.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveAutoFeedbackSpeedsPublisher = driveAutoTable
            .getStructTopic("FeedbackSpeeds", ChassisSpeeds.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveAutoCommandedSpeedsPublisher = driveAutoTable
            .getStructTopic("CommandedSpeeds", ChassisSpeeds.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveAutoMeasuredSpeedsPublisher = driveAutoTable
            .getStructTopic("MeasuredSpeeds", ChassisSpeeds.struct).publish();
    private final DoubleArrayPublisher driveAutoModuleForcesXPublisher = driveAutoTable.getDoubleArrayTopic("ModuleForcesX")
            .publish();
    private final DoubleArrayPublisher driveAutoModuleForcesYPublisher = driveAutoTable.getDoubleArrayTopic("ModuleForcesY")
            .publish();
    private final NetworkTable driveAutoStartupTable = driveAutoTable.getSubTable("Startup");
    private final DoublePublisher driveAutoInitTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitTimestampSec", "s");
    private final DoublePublisher driveAutoCommandScheduledTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "CommandScheduledTimestampSec", "s");
    private final DoublePublisher driveAutoSelectedCommandStartTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "SelectedCommandStartTimestampSec", "s");
    private final DoublePublisher driveAutoRoutineFirstPollTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "RoutineFirstPollTimestampSec", "s");
    private final DoublePublisher driveAutoRoutineStartTriggerTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "RoutineStartTriggerTimestampSec", "s");
    private final DoublePublisher driveAutoSequenceStartTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "SequenceStartTimestampSec", "s");
    private final DoublePublisher driveAutoResetOdometryTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "ResetOdometryTimestampSec", "s");
    private final DoublePublisher driveAutoDeployIntakeTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "DeployIntakeTimestampSec", "s");
    private final DoublePublisher driveAutoFirstTrajectoryCommandInitTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstTrajectoryCommandInitTimestampSec", "s");
    private final DoublePublisher driveAutoFirstPathSampleTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstPathSampleTimestampSec", "s");
    private final DoublePublisher driveAutoFirstMotionTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstMotionTimestampSec", "s");
    private final DoublePublisher driveAutoFirstMeasuredMovementTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstMeasuredMovementTimestampSec", "s");
    private final DoublePublisher driveAutoOdomCenterlineTouchTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "OdomCenterlineTouchTimestampSec", "s");
    private final DoublePublisher driveAutoCenterlineTouchTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "CenterlineTouchTimestampSec", "s");
    private final DoublePublisher driveAutoInitToCommandSchedulePublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToCommandScheduleSec", "s");
    private final DoublePublisher driveAutoInitToSelectedCommandStartPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToSelectedCommandStartSec", "s");
    private final DoublePublisher driveAutoInitToRoutineFirstPollPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToRoutineFirstPollSec", "s");
    private final DoublePublisher driveAutoInitToRoutineStartTriggerPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToRoutineStartTriggerSec", "s");
    private final DoublePublisher driveAutoInitToSequenceStartPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToSequenceStartSec", "s");
    private final DoublePublisher driveAutoInitToResetOdometryPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToResetOdometrySec", "s");
    private final DoublePublisher driveAutoInitToDeployIntakePublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToDeployIntakeSec", "s");
    private final DoublePublisher driveAutoInitToFirstTrajectoryCommandInitPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToFirstTrajectoryCommandInitSec", "s");
    private final DoublePublisher driveAutoInitToFirstPathSamplePublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToFirstPathSampleSec", "s");
    private final DoublePublisher driveAutoInitToFirstMotionPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToFirstMotionSec", "s");
    private final DoublePublisher driveAutoInitToFirstMeasuredMovementPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToFirstMeasuredMovementSec", "s");
    private final DoublePublisher driveAutoInitToOdomCenterlineTouchPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToOdomCenterlineTouchSec", "s");
    private final DoublePublisher driveAutoInitToCenterlineTouchPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "AutoInitToCenterlineTouchSec", "s");
    private final DoublePublisher driveAutoFirstMotionTranslationPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstMotionTranslationMps", "m/s");
    private final DoublePublisher driveAutoFirstMotionOmegaPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstMotionOmegaRadPerSec", "rad/s");
    private final DoublePublisher driveAutoFirstMeasuredMovementTranslationPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstMeasuredMovementTranslationMps", "m/s");
    private final DoublePublisher driveAutoFirstMeasuredMovementOmegaPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "FirstMeasuredMovementOmegaRadPerSec", "rad/s");
    private final DoublePublisher driveAutoOdomCenterlineTouchXPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "OdomCenterlineTouchXM", "m");
    private final DoublePublisher driveAutoOdomCenterlineTouchYPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "OdomCenterlineTouchYM", "m");
    private final DoublePublisher driveAutoOdomCenterlineTouchHeadingPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "OdomCenterlineTouchHeadingDeg", "deg");
    private final DoublePublisher driveAutoCenterlineTouchXPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "CenterlineTouchXM", "m");
    private final DoublePublisher driveAutoCenterlineTouchYPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "CenterlineTouchYM", "m");
    private final DoublePublisher driveAutoCenterlineTouchHeadingPublisher = TelemetryUtil.publishDouble(
            driveAutoStartupTable, "CenterlineTouchHeadingDeg", "deg");
    private final BooleanPublisher driveAutoCommandScheduledSeenPublisher = driveAutoStartupTable
            .getBooleanTopic("CommandScheduledSeen").publish();
    private final BooleanPublisher driveAutoFirstPathSampleSeenPublisher = driveAutoStartupTable
            .getBooleanTopic("FirstPathSampleSeen").publish();
    private final BooleanPublisher driveAutoFirstMotionSeenPublisher = driveAutoStartupTable
            .getBooleanTopic("FirstMotionSeen").publish();
    private final BooleanPublisher driveAutoFirstMeasuredMovementSeenPublisher = driveAutoStartupTable
            .getBooleanTopic("FirstMeasuredMovementSeen").publish();
    private final BooleanPublisher driveAutoOdomCenterlineTouchSeenPublisher = driveAutoStartupTable
            .getBooleanTopic("OdomCenterlineTouchSeen").publish();
    private final BooleanPublisher driveAutoCenterlineTouchSeenPublisher = driveAutoStartupTable
            .getBooleanTopic("CenterlineTouchSeen").publish();

    private final NetworkTable intakeTable = telemetryTable.getSubTable("Intake");
    private final StructPublisher<IntakeSnapshot> intakeStatePublisher = intakeTable
            .getStructTopic("State", IntakeSnapshot.struct).publish();
    private final NetworkTable intakePivotTable = intakeTable.getSubTable("Pivot");
    private final DoublePublisher intakePivotStatorCurrentPublisher = TelemetryUtil.publishDouble(
            intakePivotTable, "StatorCurrentAmps", "A");
    private final NetworkTable intakeLeftRollerTable = intakeTable.getSubTable("LeftRoller");
    private final DoublePublisher intakeLeftRollerStatorCurrentPublisher = TelemetryUtil.publishDouble(
            intakeLeftRollerTable, "StatorCurrentAmps", "A");
    private final NetworkTable intakeRightRollerTable = intakeTable.getSubTable("RightRoller");
    private final DoublePublisher intakeRightRollerStatorCurrentPublisher = TelemetryUtil.publishDouble(
            intakeRightRollerTable, "StatorCurrentAmps", "A");

    private final NetworkTable shooterTable = telemetryTable.getSubTable("Shooter");
    private final StructPublisher<ShooterSnapshot> shooterStatePublisher = shooterTable
            .getStructTopic("State", ShooterSnapshot.struct).publish();
    private final StringPublisher shooterStateLabelPublisher = shooterTable.getStringTopic("StateLabel").publish();
    private final NetworkTable shooterSolverTable = shooterTable.getSubTable("Solver");
    private final AutoAimSolverPublishers shooterSolverPublishers = createAutoAimSolverPublishers(shooterSolverTable);
    private final AutoAimSolverPublishers shooterSolverNonSotmPublishers = createAutoAimSolverPublishers(
            shooterSolverTable.getSubTable("NonSOTM"));
    private final AutoAimSolverPublishers shooterSolverSotmPublishers = createAutoAimSolverPublishers(
            shooterSolverTable.getSubTable("SOTM"));
    private final NetworkTable shooterCharacterizationTable = shooterTable.getSubTable("Characterization");
    private final StructPublisher<ShooterCharacterizationSnapshot> shooterCharacterizationPublisher = shooterCharacterizationTable
            .getStructTopic("State", ShooterCharacterizationSnapshot.struct).publish();
    private final BooleanPublisher shooterCharacterizationActivePublisher = shooterCharacterizationTable
            .getBooleanTopic("Active").publish();
    private final BooleanPublisher shooterCharacterizationCompletePublisher = shooterCharacterizationTable
            .getBooleanTopic("Complete").publish();
    private final BooleanPublisher shooterCharacterizationAbortedPublisher = shooterCharacterizationTable
            .getBooleanTopic("Aborted").publish();
    private final IntegerPublisher shooterCharacterizationRunIdPublisher = shooterCharacterizationTable
            .getIntegerTopic("RunId").publish();
    private final IntegerPublisher shooterCharacterizationModeCodePublisher = shooterCharacterizationTable
            .getIntegerTopic("ModeCode").publish();
    private final StringPublisher shooterCharacterizationModeLabelPublisher = shooterCharacterizationTable
            .getStringTopic("ModeLabel").publish();
    private final IntegerPublisher shooterCharacterizationSegmentIndexPublisher = shooterCharacterizationTable
            .getIntegerTopic("SegmentIndex").publish();
    private final IntegerPublisher shooterCharacterizationSegmentCountPublisher = shooterCharacterizationTable
            .getIntegerTopic("SegmentCount").publish();
    private final StringPublisher shooterCharacterizationAbortReasonPublisher = shooterCharacterizationTable
            .getStringTopic("AbortReason").publish();

    private final NetworkTable hopperTable = telemetryTable.getSubTable("Hopper");
    private final StructPublisher<HopperSnapshot> hopperStatePublisher = hopperTable
            .getStructTopic("State", HopperSnapshot.struct).publish();
    private final StructPublisher<HopperPreloadSnapshot> hopperPreloadPublisher = hopperTable
            .getStructTopic("Preload", HopperPreloadSnapshot.struct).publish();
    private final BooleanPublisher hopperTopBeamBreakRawBlockedPublisher = hopperTable
            .getBooleanTopic("TopBeamBreakRawBlocked").publish();
    private final BooleanPublisher hopperTopBeamBreakBlockedPublisher = hopperTable
            .getBooleanTopic("TopBeamBreakBlocked").publish();
    private final NetworkTable hopperFeederTable = hopperTable.getSubTable("Feeder");
    private final DoublePublisher hopperFeederCommandedVoltsPublisher = TelemetryUtil.publishDouble(
            hopperFeederTable, "CommandedVolts", "V");
    private final NetworkTable hopperFeederLeftTable = hopperFeederTable.getSubTable("Left");
    private final DoublePublisher hopperFeederLeftVelocityPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "VelocityRps", "rps");
    private final DoublePublisher hopperFeederLeftSupplyCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "SupplyCurrentAmps", "A");
    private final DoublePublisher hopperFeederLeftStatorCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "StatorCurrentAmps", "A");
    private final DoublePublisher hopperFeederLeftAppliedVoltsPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "AppliedVolts", "V");
    private final NetworkTable hopperFeederRightTable = hopperFeederTable.getSubTable("Right");
    private final DoublePublisher hopperFeederRightVelocityPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "VelocityRps", "rps");
    private final DoublePublisher hopperFeederRightSupplyCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "SupplyCurrentAmps", "A");
    private final DoublePublisher hopperFeederRightStatorCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "StatorCurrentAmps", "A");
    private final DoublePublisher hopperFeederRightAppliedVoltsPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "AppliedVolts", "V");
    private final NetworkTable hopperAutoStageTable = hopperTable.getSubTable("AutoStage");
    private final StructPublisher<HopperAutoStageSnapshot> hopperAutoStagePublisher = hopperAutoStageTable
            .getStructTopic("State", HopperAutoStageSnapshot.struct).publish();
    private final StringPublisher hopperAutoStagePhaseLabelPublisher = hopperAutoStageTable
            .getStringTopic("PhaseLabel").publish();
    private final StringPublisher hopperAutoStageReasonLabelPublisher = hopperAutoStageTable
            .getStringTopic("ReasonLabel").publish();

    private final NetworkTable superstructureTable = telemetryTable.getSubTable("Superstructure");
    private final StructPublisher<SuperstructureSnapshot> superstructureStatePublisher = superstructureTable
            .getStructTopic("State", SuperstructureSnapshot.struct).publish();
    private final BooleanPublisher superstructureReleaseBlockedPublisher = superstructureTable
            .getBooleanTopic("ReleaseBlocked").publish();
    private final StringPublisher superstructureReleaseBlockReasonPublisher = superstructureTable
            .getStringTopic("ReleaseBlockReason").publish();
    private final NetworkTable superstructureReleaseTable = superstructureTable.getSubTable("Release");
    private final StructPublisher<ShotReleaseSnapshot> superstructureReleaseStatePublisher = superstructureReleaseTable
            .getStructTopic("State", ShotReleaseSnapshot.struct).publish();
    private final BooleanPublisher superstructureReleaseConditionBlockedPublisher = superstructureReleaseTable
            .getBooleanTopic("Blocked").publish();
    private final StringPublisher superstructureReleaseConditionBlockReasonPublisher = superstructureReleaseTable
            .getStringTopic("BlockReason").publish();

    private final NetworkTable visionTable = telemetryTable.getSubTable("Vision");
    private final StructPublisher<VisionSnapshot> visionStatePublisher = visionTable
            .getStructTopic("State", VisionSnapshot.struct).publish();
    private final IntegerPublisher visionValidationStatusPublisher = visionTable.getIntegerTopic("ValidationStatusCode")
            .publish();

    private final NetworkTable simTable = telemetryTable.getSubTable("Sim");
    private final StructPublisher<SimSnapshot> simStatePublisher = simTable.getStructTopic("State", SimSnapshot.struct)
            .publish();
    private final StructArrayPublisher<Pose3d> simBallPosesPublisher = simTable.getStructArrayTopic("BallPoses", Pose3d.struct)
            .publish();
    private final StructPublisher<Pose3d> simTargetPosePublisher = simTable.getStructTopic("TargetPose", Pose3d.struct)
            .publish();

    private final NetworkTable controlsTable = telemetryTable.getSubTable("Controls");
    private final BooleanPublisher driveAutoAimActivePublisher = controlsTable.getBooleanTopic("Drive/AutoAimActive").publish();
    private final DoublePublisher driveAutoAimRotCmdPublisher = TelemetryUtil.publishDouble(controlsTable,
            "Drive/AutoAimRotCmdRadPerSec", "rad/s");
    private final DoublePublisher driveAutoAimTransLimitPublisher = TelemetryUtil.publishDouble(controlsTable,
            "Drive/AutoAimTransLimitMps", "m/s");
    private final StructPublisher<YShotSnapshot> yShotPublisher = controlsTable.getStructTopic("YShot/State", YShotSnapshot.struct)
            .publish();
    private final NetworkTable timedShotTable = controlsTable.getSubTable("TimedShot");
    private final StructPublisher<TimedShotSnapshot> timedShotPublisher = timedShotTable
            .getStructTopic("State", TimedShotSnapshot.struct).publish();
    private final StringPublisher timedShotReasonLabelPublisher = timedShotTable.getStringTopic("ReasonLabel").publish();

    private final NetworkTable practiceTimerTable = telemetryTable.getSubTable("PracticeTimer");
    private final StructPublisher<PracticeTimerSnapshot> practiceTimerStatePublisher = practiceTimerTable
            .getStructTopic("State", PracticeTimerSnapshot.struct).publish();
    private final IntegerPublisher practiceTimerPhaseCodePublisher = practiceTimerTable.getIntegerTopic("PhaseCode").publish();
    private final StringPublisher practiceTimerPhaseLabelPublisher = practiceTimerTable.getStringTopic("PhaseLabel").publish();
    private final BooleanPublisher practiceTimerActivePublisher = practiceTimerTable.getBooleanTopic("Active").publish();
    private final BooleanPublisher practiceTimerCompletePublisher = practiceTimerTable.getBooleanTopic("Complete").publish();
    private final DoublePublisher practiceTimerPhaseRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "PhaseRemainingSec", "s");
    private final DoublePublisher practiceTimerTotalRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "TotalRemainingSec", "s");
    private final IntegerPublisher practiceTimerRunIdPublisher = practiceTimerTable.getIntegerTopic("RunId").publish();
    private final BooleanPublisher practiceTimerUsingDsMatchTimePublisher = practiceTimerTable
            .getBooleanTopic("UsingDsMatchTime").publish();
    private final IntegerPublisher practiceTimerZoneCodePublisher = practiceTimerTable.getIntegerTopic("ZoneCode").publish();
    private final StringPublisher practiceTimerZoneLabelPublisher = practiceTimerTable.getStringTopic("ZoneLabel").publish();
    private final BooleanPublisher practiceTimerZoneActivePublisher = practiceTimerTable.getBooleanTopic("ZoneActive").publish();
    private final BooleanPublisher practiceTimerZoneKnownPublisher = practiceTimerTable.getBooleanTopic("ZoneKnown").publish();
    private final BooleanPublisher practiceTimerEndgamePublisher = practiceTimerTable.getBooleanTopic("Endgame").publish();
    private final DoublePublisher practiceTimerZoneRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "ZoneRemainingSec", "s");
    private final DoublePublisher practiceTimerTeleopRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "TeleopRemainingSec", "s");

    private final Map<String, VisionCameraPublishers> visionCameraPublishers = new HashMap<>();

    private final Map<String, StructPublisher<Translation2d>> translationPublishers = new HashMap<>();
    private final Map<String, StructPublisher<Rotation2d>> rotationPublishers = new HashMap<>();
    private final Map<String, StructArrayPublisher<Pose2d>> pose2dArrayPublishers = new HashMap<>();
    private final Map<String, StructArrayPublisher<Pose3d>> pose3dArrayPublishers = new HashMap<>();

    private int lastVisionValidationStatusCode = VALIDATION_OK;
    private static final double kHeadingLineLengthMeters = Math.hypot(FieldConstants.fieldLength, FieldConstants.fieldWidth);
    private double driveAutoInitTimestampSec = Double.NaN;
    private double driveAutoCommandScheduledTimestampSec = Double.NaN;
    private double driveAutoSelectedCommandStartTimestampSec = Double.NaN;
    private double driveAutoRoutineFirstPollTimestampSec = Double.NaN;
    private double driveAutoRoutineStartTriggerTimestampSec = Double.NaN;
    private double driveAutoSequenceStartTimestampSec = Double.NaN;
    private double driveAutoResetOdometryTimestampSec = Double.NaN;
    private double driveAutoDeployIntakeTimestampSec = Double.NaN;
    private double driveAutoFirstTrajectoryCommandInitTimestampSec = Double.NaN;
    private double driveAutoFirstPathSampleTimestampSec = Double.NaN;
    private double driveAutoFirstMotionTimestampSec = Double.NaN;
    private double driveAutoFirstMeasuredMovementTimestampSec = Double.NaN;
    private double driveAutoOdomCenterlineTouchTimestampSec = Double.NaN;
    private double driveAutoCenterlineTouchTimestampSec = Double.NaN;
    private double driveAutoFirstMotionTranslationMps = Double.NaN;
    private double driveAutoFirstMotionOmegaRadPerSec = Double.NaN;
    private double driveAutoFirstMeasuredMovementTranslationMps = Double.NaN;
    private double driveAutoFirstMeasuredMovementOmegaRadPerSec = Double.NaN;
    private Pose2d driveAutoOdomCenterlineTouchPose = Pose2d.kZero;
    private Pose2d driveAutoCenterlineTouchPose = Pose2d.kZero;
    private Pose2d lastDrivePoseForAutoOdomTracking = Pose2d.kZero;
    private double lastDrivePoseTimestampForAutoOdomTrackingSec = Double.NaN;
    private boolean autoOdomTrackingPoseSeen = false;

    private TelemetryManager() {
    }

    public void publishDrivePose(double timestampSec, Pose2d pose) {
        drivePosePublisher.set(pose);
        publishDriveAimGeometry(pose);
        trackAutoOdomCenterlineTouch(timestampSec, pose);
    }

    public void publishDrivePose(Pose2d pose) {
        publishDrivePose(Double.NaN, pose);
    }

    public void publishDriveSpeeds(double timestampSec, ChassisSpeeds speeds) {
        driveSpeedsPublisher.set(speeds);
        double measuredTranslationMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        if (DriverStation.isAutonomous()
                && DriverStation.isEnabled()
                && Double.isFinite(driveAutoInitTimestampSec)
                && Double.isNaN(driveAutoFirstMeasuredMovementTimestampSec)
                && (measuredTranslationMps >= kAutoStartupMeasuredTranslationThresholdMps
                        || Math.abs(speeds.omegaRadiansPerSecond) >= kAutoStartupMeasuredOmegaThresholdRadPerSec)) {
            markDriveAutoFirstMeasuredMovement(timestampSec, speeds);
        }
    }

    public void publishDriveSpeeds(ChassisSpeeds speeds) {
        publishDriveSpeeds(Double.NaN, speeds);
    }

    public void publishDriveModuleStates(SwerveModuleState[] states) {
        driveModuleStatesPublisher.set(states);
    }

    public void publishDriveModuleTargets(SwerveModuleState[] targets) {
        driveModuleTargetsPublisher.set(targets);
    }

    public void publishDriveModulePositions(SwerveModulePosition[] positions) {
        driveModulePositionsPublisher.set(positions);
    }

    public void publishDriveModuleElectrical(double[] driveVelocityRps, double[] driveStatorCurrentAmps) {
        driveModuleVelocityPublisher.set(driveVelocityRps);
        driveModuleStatorCurrentPublisher.set(driveStatorCurrentAmps);
    }

    public void publishDriveCurrentLimits(
            double activeStatorCurrentLimitAmps,
            double nominalStatorCurrentLimitAmps,
            double turboStatorCurrentLimitAmps,
            boolean turboEnabled) {
        driveCurrentLimitActivePublisher.set(activeStatorCurrentLimitAmps);
        driveCurrentLimitNominalPublisher.set(nominalStatorCurrentLimitAmps);
        driveCurrentLimitTurboPublisher.set(turboStatorCurrentLimitAmps);
        driveCurrentLimitTurboEnabledPublisher.set(turboEnabled);
    }

    public void publishDriveStatus(DriveStatusSnapshot snapshot) {
        driveStatusPublisher.set(snapshot);
    }

    public void publishDriveAutoPathActive(boolean active) {
        driveAutoActivePublisher.set(active);
    }

    public void publishDriveAutoPathSample(
            double sampleTimestampSec,
            Pose2d desiredPose,
            Pose2d measuredPose,
            Translation2d poseError,
            double headingErrorRad,
            ChassisSpeeds feedforwardSpeeds,
            ChassisSpeeds feedbackSpeeds,
            ChassisSpeeds commandedSpeeds,
            ChassisSpeeds measuredSpeeds,
            double[] moduleForcesX,
            double[] moduleForcesY) {
        driveAutoActivePublisher.set(true);
        driveAutoLastSampleTimestampPublisher.set(sampleTimestampSec);
        driveAutoDesiredPosePublisher.set(desiredPose);
        driveAutoMeasuredPosePublisher.set(measuredPose);
        driveAutoPoseErrorPublisher.set(poseError);
        driveAutoPoseErrorNormPublisher.set(poseError.getNorm());
        driveAutoHeadingErrorPublisher.set(headingErrorRad);
        driveAutoFeedforwardSpeedsPublisher.set(feedforwardSpeeds);
        driveAutoFeedbackSpeedsPublisher.set(feedbackSpeeds);
        driveAutoCommandedSpeedsPublisher.set(commandedSpeeds);
        driveAutoMeasuredSpeedsPublisher.set(measuredSpeeds);
        driveAutoModuleForcesXPublisher.set(moduleForcesX);
        driveAutoModuleForcesYPublisher.set(moduleForcesY);
    }

    public void resetDriveAutoStartupProfile() {
        driveAutoInitTimestampSec = Double.NaN;
        driveAutoCommandScheduledTimestampSec = Double.NaN;
        driveAutoSelectedCommandStartTimestampSec = Double.NaN;
        driveAutoRoutineFirstPollTimestampSec = Double.NaN;
        driveAutoRoutineStartTriggerTimestampSec = Double.NaN;
        driveAutoSequenceStartTimestampSec = Double.NaN;
        driveAutoResetOdometryTimestampSec = Double.NaN;
        driveAutoDeployIntakeTimestampSec = Double.NaN;
        driveAutoFirstTrajectoryCommandInitTimestampSec = Double.NaN;
        driveAutoFirstPathSampleTimestampSec = Double.NaN;
        driveAutoFirstMotionTimestampSec = Double.NaN;
        driveAutoFirstMeasuredMovementTimestampSec = Double.NaN;
        driveAutoOdomCenterlineTouchTimestampSec = Double.NaN;
        driveAutoCenterlineTouchTimestampSec = Double.NaN;
        driveAutoFirstMotionTranslationMps = Double.NaN;
        driveAutoFirstMotionOmegaRadPerSec = Double.NaN;
        driveAutoFirstMeasuredMovementTranslationMps = Double.NaN;
        driveAutoFirstMeasuredMovementOmegaRadPerSec = Double.NaN;
        driveAutoOdomCenterlineTouchPose = Pose2d.kZero;
        driveAutoCenterlineTouchPose = Pose2d.kZero;
        lastDrivePoseForAutoOdomTracking = Pose2d.kZero;
        lastDrivePoseTimestampForAutoOdomTrackingSec = Double.NaN;
        autoOdomTrackingPoseSeen = false;
        publishDriveAutoStartupProfile();
    }

    public void markDriveAutoInit(double timestampSec) {
        driveAutoInitTimestampSec = timestampSec;
        publishDriveAutoStartupProfile();
    }

    public void markDriveAutoCommandScheduled(double timestampSec) {
        if (Double.isNaN(driveAutoCommandScheduledTimestampSec)) {
            driveAutoCommandScheduledTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoSelectedCommandStart(double timestampSec) {
        if (Double.isNaN(driveAutoSelectedCommandStartTimestampSec)) {
            driveAutoSelectedCommandStartTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoRoutineFirstPoll(double timestampSec) {
        if (Double.isNaN(driveAutoRoutineFirstPollTimestampSec)) {
            driveAutoRoutineFirstPollTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoRoutineStartTrigger(double timestampSec) {
        if (Double.isNaN(driveAutoRoutineStartTriggerTimestampSec)) {
            driveAutoRoutineStartTriggerTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoSequenceStart(double timestampSec) {
        if (Double.isNaN(driveAutoSequenceStartTimestampSec)) {
            driveAutoSequenceStartTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoResetOdometry(double timestampSec) {
        if (Double.isNaN(driveAutoResetOdometryTimestampSec)) {
            driveAutoResetOdometryTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoDeployIntake(double timestampSec) {
        if (Double.isNaN(driveAutoDeployIntakeTimestampSec)) {
            driveAutoDeployIntakeTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoFirstTrajectoryCommandInit(double timestampSec) {
        if (Double.isNaN(driveAutoFirstTrajectoryCommandInitTimestampSec)) {
            driveAutoFirstTrajectoryCommandInitTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoFirstPathSample(double timestampSec) {
        if (Double.isNaN(driveAutoFirstPathSampleTimestampSec)) {
            driveAutoFirstPathSampleTimestampSec = timestampSec;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoFirstMotionCommand(double timestampSec, ChassisSpeeds commandedSpeeds) {
        if (Double.isNaN(driveAutoFirstMotionTimestampSec)) {
            driveAutoFirstMotionTimestampSec = timestampSec;
            driveAutoFirstMotionTranslationMps = Math.hypot(
                    commandedSpeeds.vxMetersPerSecond,
                    commandedSpeeds.vyMetersPerSecond);
            driveAutoFirstMotionOmegaRadPerSec = commandedSpeeds.omegaRadiansPerSecond;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoFirstMeasuredMovement(double timestampSec, ChassisSpeeds measuredSpeeds) {
        if (Double.isNaN(driveAutoFirstMeasuredMovementTimestampSec)) {
            driveAutoFirstMeasuredMovementTimestampSec = timestampSec;
            driveAutoFirstMeasuredMovementTranslationMps = Math.hypot(
                    measuredSpeeds.vxMetersPerSecond,
                    measuredSpeeds.vyMetersPerSecond);
            driveAutoFirstMeasuredMovementOmegaRadPerSec = measuredSpeeds.omegaRadiansPerSecond;
            publishDriveAutoStartupProfile();
        }
    }

    public void markDriveAutoOdomCenterlineTouch(double timestampSec, Pose2d pose) {
        if (Double.isNaN(driveAutoOdomCenterlineTouchTimestampSec)) {
            driveAutoOdomCenterlineTouchTimestampSec = timestampSec;
            driveAutoOdomCenterlineTouchPose = pose;
            publishDriveAutoStartupProfile();
            if (RobotBase.isReal()) {
                double autoInitToFirstMeasuredMovementSec = latencyFromAutoInit(
                        driveAutoFirstMeasuredMovementTimestampSec);
                double autoInitToOdomCenterlineTouchSec = latencyFromAutoInit(driveAutoOdomCenterlineTouchTimestampSec);
                System.out.println("==============================================");
                System.out.println("  AUTO ODOM CENTERLINE TOUCH");
                System.out.println("----------------------------------------------");
                System.out.printf("  Auto init -> first measured movement: %.3f s%n", autoInitToFirstMeasuredMovementSec);
                System.out.printf("  First measured movement translation: %.3f m/s%n",
                        driveAutoFirstMeasuredMovementTranslationMps);
                System.out.printf("  Auto init -> odom centerline touch: %.3f s%n", autoInitToOdomCenterlineTouchSec);
                System.out.printf("  Odom centerline touch pose: (%.3f, %.3f, %.1f deg)%n",
                        pose.getX(),
                        pose.getY(),
                        pose.getRotation().getDegrees());
                System.out.println("==============================================");
            }
        }
    }

    public void markDriveAutoCenterlineTouch(double timestampSec, Pose2d pose) {
        if (Double.isNaN(driveAutoCenterlineTouchTimestampSec)) {
            driveAutoCenterlineTouchTimestampSec = timestampSec;
            driveAutoCenterlineTouchPose = pose;
            publishDriveAutoStartupProfile();
        }
    }

    private void publishDriveAutoStartupProfile() {
        driveAutoInitTimestampPublisher.set(driveAutoInitTimestampSec);
        driveAutoCommandScheduledTimestampPublisher.set(driveAutoCommandScheduledTimestampSec);
        driveAutoSelectedCommandStartTimestampPublisher.set(driveAutoSelectedCommandStartTimestampSec);
        driveAutoRoutineFirstPollTimestampPublisher.set(driveAutoRoutineFirstPollTimestampSec);
        driveAutoRoutineStartTriggerTimestampPublisher.set(driveAutoRoutineStartTriggerTimestampSec);
        driveAutoSequenceStartTimestampPublisher.set(driveAutoSequenceStartTimestampSec);
        driveAutoResetOdometryTimestampPublisher.set(driveAutoResetOdometryTimestampSec);
        driveAutoDeployIntakeTimestampPublisher.set(driveAutoDeployIntakeTimestampSec);
        driveAutoFirstTrajectoryCommandInitTimestampPublisher.set(driveAutoFirstTrajectoryCommandInitTimestampSec);
        driveAutoFirstPathSampleTimestampPublisher.set(driveAutoFirstPathSampleTimestampSec);
        driveAutoFirstMotionTimestampPublisher.set(driveAutoFirstMotionTimestampSec);
        driveAutoFirstMeasuredMovementTimestampPublisher.set(driveAutoFirstMeasuredMovementTimestampSec);
        driveAutoOdomCenterlineTouchTimestampPublisher.set(driveAutoOdomCenterlineTouchTimestampSec);
        driveAutoCenterlineTouchTimestampPublisher.set(driveAutoCenterlineTouchTimestampSec);
        driveAutoInitToCommandSchedulePublisher.set(
                latencyFromAutoInit(driveAutoCommandScheduledTimestampSec));
        driveAutoInitToSelectedCommandStartPublisher.set(
                latencyFromAutoInit(driveAutoSelectedCommandStartTimestampSec));
        driveAutoInitToRoutineFirstPollPublisher.set(
                latencyFromAutoInit(driveAutoRoutineFirstPollTimestampSec));
        driveAutoInitToRoutineStartTriggerPublisher.set(
                latencyFromAutoInit(driveAutoRoutineStartTriggerTimestampSec));
        driveAutoInitToSequenceStartPublisher.set(
                latencyFromAutoInit(driveAutoSequenceStartTimestampSec));
        driveAutoInitToResetOdometryPublisher.set(
                latencyFromAutoInit(driveAutoResetOdometryTimestampSec));
        driveAutoInitToDeployIntakePublisher.set(
                latencyFromAutoInit(driveAutoDeployIntakeTimestampSec));
        driveAutoInitToFirstTrajectoryCommandInitPublisher.set(
                latencyFromAutoInit(driveAutoFirstTrajectoryCommandInitTimestampSec));
        driveAutoInitToFirstPathSamplePublisher.set(
                latencyFromAutoInit(driveAutoFirstPathSampleTimestampSec));
        driveAutoInitToFirstMotionPublisher.set(
                latencyFromAutoInit(driveAutoFirstMotionTimestampSec));
        driveAutoInitToFirstMeasuredMovementPublisher.set(
                latencyFromAutoInit(driveAutoFirstMeasuredMovementTimestampSec));
        driveAutoInitToOdomCenterlineTouchPublisher.set(
                latencyFromAutoInit(driveAutoOdomCenterlineTouchTimestampSec));
        driveAutoInitToCenterlineTouchPublisher.set(
                latencyFromAutoInit(driveAutoCenterlineTouchTimestampSec));
        driveAutoFirstMotionTranslationPublisher.set(driveAutoFirstMotionTranslationMps);
        driveAutoFirstMotionOmegaPublisher.set(driveAutoFirstMotionOmegaRadPerSec);
        driveAutoFirstMeasuredMovementTranslationPublisher.set(driveAutoFirstMeasuredMovementTranslationMps);
        driveAutoFirstMeasuredMovementOmegaPublisher.set(driveAutoFirstMeasuredMovementOmegaRadPerSec);
        driveAutoOdomCenterlineTouchXPublisher.set(driveAutoOdomCenterlineTouchPose.getX());
        driveAutoOdomCenterlineTouchYPublisher.set(driveAutoOdomCenterlineTouchPose.getY());
        driveAutoOdomCenterlineTouchHeadingPublisher.set(driveAutoOdomCenterlineTouchPose.getRotation().getDegrees());
        driveAutoCenterlineTouchXPublisher.set(driveAutoCenterlineTouchPose.getX());
        driveAutoCenterlineTouchYPublisher.set(driveAutoCenterlineTouchPose.getY());
        driveAutoCenterlineTouchHeadingPublisher.set(driveAutoCenterlineTouchPose.getRotation().getDegrees());
        driveAutoCommandScheduledSeenPublisher.set(Double.isFinite(driveAutoCommandScheduledTimestampSec));
        driveAutoFirstPathSampleSeenPublisher.set(Double.isFinite(driveAutoFirstPathSampleTimestampSec));
        driveAutoFirstMotionSeenPublisher.set(Double.isFinite(driveAutoFirstMotionTimestampSec));
        driveAutoFirstMeasuredMovementSeenPublisher.set(Double.isFinite(driveAutoFirstMeasuredMovementTimestampSec));
        driveAutoOdomCenterlineTouchSeenPublisher.set(Double.isFinite(driveAutoOdomCenterlineTouchTimestampSec));
        driveAutoCenterlineTouchSeenPublisher.set(Double.isFinite(driveAutoCenterlineTouchTimestampSec));
    }

    private void trackAutoOdomCenterlineTouch(double timestampSec, Pose2d pose) {
        if (!RobotBase.isReal()
                || !DriverStation.isAutonomous()
                || !DriverStation.isEnabled()
                || !Double.isFinite(driveAutoInitTimestampSec)
                || Double.isNaN(timestampSec)
                || Double.isFinite(driveAutoOdomCenterlineTouchTimestampSec)) {
            return;
        }

        double touchThresholdX = FieldConstants.LinesVertical.center - kAutoCenterlineTouchDistanceM;
        if (!autoOdomTrackingPoseSeen || !Double.isFinite(lastDrivePoseTimestampForAutoOdomTrackingSec)) {
            autoOdomTrackingPoseSeen = true;
            lastDrivePoseForAutoOdomTracking = pose;
            lastDrivePoseTimestampForAutoOdomTrackingSec = timestampSec;
            if (pose.getX() >= touchThresholdX) {
                markDriveAutoOdomCenterlineTouch(timestampSec, pose);
            }
            return;
        }

        double previousX = lastDrivePoseForAutoOdomTracking.getX();
        double currentX = pose.getX();
        if (previousX < touchThresholdX && currentX >= touchThresholdX) {
            double fraction = interpolationFraction(previousX, currentX, touchThresholdX);
            markDriveAutoOdomCenterlineTouch(
                    interpolate(lastDrivePoseTimestampForAutoOdomTrackingSec, timestampSec, fraction),
                    interpolatePose(lastDrivePoseForAutoOdomTracking, pose, fraction));
        }

        lastDrivePoseForAutoOdomTracking = pose;
        lastDrivePoseTimestampForAutoOdomTrackingSec = timestampSec;
    }

    private static double interpolationFraction(double start, double end, double target) {
        double denominator = end - start;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, (target - start) / denominator));
    }

    private static double interpolate(double start, double end, double fraction) {
        return start + (end - start) * fraction;
    }

    private static Pose2d interpolatePose(Pose2d start, Pose2d end, double fraction) {
        return new Pose2d(
                interpolate(start.getX(), end.getX(), fraction),
                interpolate(start.getY(), end.getY(), fraction),
                start.getRotation().interpolate(end.getRotation(), fraction));
    }

    private double latencyFromAutoInit(double timestampSec) {
        if (!Double.isFinite(driveAutoInitTimestampSec) || !Double.isFinite(timestampSec)) {
            return Double.NaN;
        }
        return timestampSec - driveAutoInitTimestampSec;
    }

    public void publishIntakeState(IntakeSnapshot snapshot) {
        intakeStatePublisher.set(snapshot);
        intakePivotStatorCurrentPublisher.set(snapshot.pivotStatorCurrentAmps());
        intakeLeftRollerStatorCurrentPublisher.set(snapshot.leftRollerStatorCurrentAmps());
        intakeRightRollerStatorCurrentPublisher.set(snapshot.rightRollerStatorCurrentAmps());
    }

    public void publishShooterState(ShooterSnapshot snapshot, String stateLabel) {
        shooterStatePublisher.set(snapshot);
        shooterStateLabelPublisher.set(stateLabel);
    }

    public void publishAutoAimSolver(AutoAimSolverSnapshot snapshot, String modeLabel) {
        publishAutoAimSolver(shooterSolverPublishers, snapshot, modeLabel);
    }

    public void publishAutoAimSolverGeometry(Translation3d fieldTarget, Translation2d lookaheadTarget) {
        publishAutoAimSolverGeometry(shooterSolverPublishers, fieldTarget, lookaheadTarget);
    }

    public void publishAutoAimNonSotmSolver(AutoAimSolverSnapshot snapshot, String modeLabel) {
        publishAutoAimSolver(shooterSolverNonSotmPublishers, snapshot, modeLabel);
    }

    public void publishAutoAimNonSotmGeometry(Translation3d fieldTarget, Translation2d lookaheadTarget) {
        publishAutoAimSolverGeometry(shooterSolverNonSotmPublishers, fieldTarget, lookaheadTarget);
    }

    public void publishAutoAimSotmSolver(AutoAimSolverSnapshot snapshot, String modeLabel) {
        publishAutoAimSolver(shooterSolverSotmPublishers, snapshot, modeLabel);
    }

    public void publishAutoAimSotmGeometry(Translation3d fieldTarget, Translation2d lookaheadTarget) {
        publishAutoAimSolverGeometry(shooterSolverSotmPublishers, fieldTarget, lookaheadTarget);
    }

    public void publishShooterCharacterization(ShooterCharacterizationSnapshot snapshot) {
        shooterCharacterizationPublisher.set(snapshot);
    }

    public void publishShooterCharacterizationStatus(
            boolean active,
            boolean complete,
            boolean aborted,
            int runId,
            int modeCode,
            String modeLabel,
            int segmentIndex,
            int segmentCount,
            String abortReason) {
        shooterCharacterizationActivePublisher.set(active);
        shooterCharacterizationCompletePublisher.set(complete);
        shooterCharacterizationAbortedPublisher.set(aborted);
        shooterCharacterizationRunIdPublisher.set(runId);
        shooterCharacterizationModeCodePublisher.set(modeCode);
        shooterCharacterizationModeLabelPublisher.set(modeLabel);
        shooterCharacterizationSegmentIndexPublisher.set(segmentIndex);
        shooterCharacterizationSegmentCountPublisher.set(segmentCount);
        shooterCharacterizationAbortReasonPublisher.set(abortReason);
    }

    public void publishHopperState(HopperSnapshot snapshot) {
        hopperStatePublisher.set(snapshot);
        hopperTopBeamBreakRawBlockedPublisher.set(snapshot.topBeamBreakRawBlocked());
        hopperTopBeamBreakBlockedPublisher.set(snapshot.topBeamBreakBlocked());
    }

    public void publishHopperPreloadState(HopperPreloadSnapshot snapshot) {
        hopperPreloadPublisher.set(snapshot);
    }

    public void publishHopperAutoStageState(
            HopperAutoStageSnapshot snapshot,
            String phaseLabel,
            String reasonLabel) {
        hopperAutoStagePublisher.set(snapshot);
        hopperAutoStagePhaseLabelPublisher.set(phaseLabel);
        hopperAutoStageReasonLabelPublisher.set(reasonLabel);
    }

    public void publishHopperFeederState(
            double commandedVolts,
            double leftVelocityRps,
            double leftSupplyCurrentAmps,
            double leftStatorCurrentAmps,
            double leftAppliedVolts,
            double rightVelocityRps,
            double rightSupplyCurrentAmps,
            double rightStatorCurrentAmps,
            double rightAppliedVolts) {
        hopperFeederCommandedVoltsPublisher.set(commandedVolts);
        hopperFeederLeftVelocityPublisher.set(leftVelocityRps);
        hopperFeederLeftSupplyCurrentPublisher.set(leftSupplyCurrentAmps);
        hopperFeederLeftStatorCurrentPublisher.set(leftStatorCurrentAmps);
        hopperFeederLeftAppliedVoltsPublisher.set(leftAppliedVolts);
        hopperFeederRightVelocityPublisher.set(rightVelocityRps);
        hopperFeederRightSupplyCurrentPublisher.set(rightSupplyCurrentAmps);
        hopperFeederRightStatorCurrentPublisher.set(rightStatorCurrentAmps);
        hopperFeederRightAppliedVoltsPublisher.set(rightAppliedVolts);
    }

    public void publishSuperstructureState(SuperstructureSnapshot snapshot) {
        superstructureStatePublisher.set(snapshot);
    }

    public void publishSuperstructureReleaseState(ShotReleaseSnapshot snapshot) {
        superstructureReleaseStatePublisher.set(snapshot);
    }

    public void publishSuperstructureReleaseBlock(boolean blocked, String reason) {
        superstructureReleaseBlockedPublisher.set(blocked);
        superstructureReleaseBlockReasonPublisher.set(reason);
        superstructureReleaseConditionBlockedPublisher.set(blocked);
        superstructureReleaseConditionBlockReasonPublisher.set(reason);
    }

    public void publishVisionState(VisionSnapshot snapshot) {
        visionStatePublisher.set(snapshot);
    }

    public void publishVisionValidationStatusCode(int statusCode) {
        lastVisionValidationStatusCode = statusCode;
        visionValidationStatusPublisher.set(statusCode);
    }

    public int getVisionValidationStatusCode() {
        return lastVisionValidationStatusCode;
    }

    public void publishVisionCameraState(String cameraName, VisionCameraSnapshot snapshot) {
        getVisionCameraPublishers(cameraName).statePublisher.set(snapshot);
    }

    public void publishVisionCameraGeometry(
            String cameraName,
            Pose3d[] tagPoses,
            Pose3d cameraPose,
            Pose3d robotPose,
            Pose2d relevantPoseEstimate) {
        VisionCameraPublishers publishers = getVisionCameraPublishers(cameraName);
        publishers.tagPosesPublisher.set(tagPoses);
        if (cameraPose != null) {
            publishers.cameraPosePublisher.set(cameraPose);
        }
        if (robotPose != null) {
            publishers.robotPosePublisher.set(robotPose);
        }
        if (relevantPoseEstimate != null) {
            publishers.relevantPosePublisher.set(relevantPoseEstimate);
        }
    }

    public void publishSimState(SimSnapshot snapshot) {
        simStatePublisher.set(snapshot);
    }

    public void publishSimGeometry(Pose3d[] ballPoses, Pose3d targetPose) {
        simBallPosesPublisher.set(ballPoses);
        simTargetPosePublisher.set(targetPose);
    }

    public void publishDriveAutoAim(boolean active, double rotCmdRadPerSec, double transLimitMps) {
        driveAutoAimActivePublisher.set(active);
        driveAutoAimRotCmdPublisher.set(rotCmdRadPerSec);
        driveAutoAimTransLimitPublisher.set(transLimitMps);
    }

    public void publishYShotState(YShotSnapshot snapshot) {
        yShotPublisher.set(snapshot);
    }

    public void publishTimedShotState(TimedShotSnapshot snapshot, String reasonLabel) {
        timedShotPublisher.set(snapshot);
        timedShotReasonLabelPublisher.set(reasonLabel);
    }

    public void publishPracticeTimerState(PracticeTimerSnapshot snapshot, String phaseLabel, String zoneLabel) {
        practiceTimerStatePublisher.set(snapshot);
        practiceTimerPhaseCodePublisher.set(snapshot.phaseCode());
        practiceTimerPhaseLabelPublisher.set(phaseLabel);
        practiceTimerActivePublisher.set(snapshot.active());
        practiceTimerCompletePublisher.set(snapshot.complete());
        practiceTimerPhaseRemainingPublisher.set(snapshot.phaseRemainingSec());
        practiceTimerTotalRemainingPublisher.set(snapshot.totalRemainingSec());
        practiceTimerRunIdPublisher.set(snapshot.runId());
        practiceTimerUsingDsMatchTimePublisher.set(snapshot.usingDsMatchTime());
        practiceTimerZoneCodePublisher.set(snapshot.zoneCode());
        practiceTimerZoneLabelPublisher.set(zoneLabel);
        practiceTimerZoneActivePublisher.set(snapshot.zoneActive());
        practiceTimerZoneKnownPublisher.set(snapshot.zoneKnown());
        practiceTimerEndgamePublisher.set(snapshot.endgame());
        practiceTimerZoneRemainingPublisher.set(snapshot.zoneRemainingSec());
        practiceTimerTeleopRemainingPublisher.set(snapshot.teleopRemainingSec());
    }

    public void publishTranslation2d(String key, Translation2d translation) {
        StructPublisher<Translation2d> publisher = translationPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructTopic("Logs/" + k, Translation2d.struct).publish());
        publisher.set(translation);
    }

    public void publishRotation2d(String key, Rotation2d rotation) {
        StructPublisher<Rotation2d> publisher = rotationPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructTopic("Logs/" + k, Rotation2d.struct).publish());
        publisher.set(rotation);
    }

    public void publishPose2dArray(String key, Pose2d[] poses) {
        StructArrayPublisher<Pose2d> publisher = pose2dArrayPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructArrayTopic("Logs/" + k, Pose2d.struct).publish());
        publisher.set(poses);
    }

    public void publishPose3dArray(String key, Pose3d[] poses) {
        StructArrayPublisher<Pose3d> publisher = pose3dArrayPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructArrayTopic("Logs/" + k, Pose3d.struct).publish());
        publisher.set(poses);
    }

    private void publishDriveAimGeometry(Pose2d robotPose) {
        Translation2d hubCenter = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Pose2d hubCenterPose = new Pose2d(hubCenter, Rotation2d.kZero);
        Translation2d headingEnd = robotPose.getTranslation().plus(new Translation2d(
                robotPose.getRotation().getCos() * kHeadingLineLengthMeters,
                robotPose.getRotation().getSin() * kHeadingLineLengthMeters));
        Pose2d headingEndPose = new Pose2d(headingEnd, robotPose.getRotation());

        driveAimHeadingLinePublisher.set(new Pose2d[] { robotPose, headingEndPose });
        driveAimHubReferenceLinePublisher.set(new Pose2d[] { robotPose, hubCenterPose });
        driveAimHubCenterPublisher.set(hubCenterPose);
    }

    private void publishAutoAimSolver(AutoAimSolverPublishers publishers, AutoAimSolverSnapshot snapshot, String modeLabel) {
        publishers.statePublisher.set(snapshot);
        publishers.modeLabelPublisher.set(modeLabel);
    }

    private void publishAutoAimSolverGeometry(
            AutoAimSolverPublishers publishers,
            Translation3d fieldTarget,
            Translation2d lookaheadTarget) {
        publishers.fieldTargetPublisher.set(fieldTarget);
        publishers.lookaheadTargetPublisher.set(lookaheadTarget);
    }

    private VisionCameraPublishers getVisionCameraPublishers(String cameraName) {
        return visionCameraPublishers.computeIfAbsent(cameraName, this::createVisionCameraPublishers);
    }

    private AutoAimSolverPublishers createAutoAimSolverPublishers(NetworkTable table) {
        return new AutoAimSolverPublishers(
                table.getStructTopic("State", AutoAimSolverSnapshot.struct).publish(),
                table.getStringTopic("ModeLabel").publish(),
                table.getStructTopic("FieldTarget", Translation3d.struct).publish(),
                table.getStructTopic("LookaheadTarget", Translation2d.struct).publish());
    }

    private VisionCameraPublishers createVisionCameraPublishers(String cameraName) {
        NetworkTable cameraTable = visionTable.getSubTable(cameraName);
        return new VisionCameraPublishers(
                cameraTable.getStructTopic("State", VisionCameraSnapshot.struct).publish(),
                cameraTable.getStructArrayTopic("TagPoses", Pose3d.struct).publish(),
                cameraTable.getStructTopic("CameraPose", Pose3d.struct).publish(),
                cameraTable.getStructTopic("RobotPose", Pose3d.struct).publish(),
                cameraTable.getStructTopic("RelevantPoseEstimate", Pose2d.struct).publish());
    }

    private record AutoAimSolverPublishers(
            StructPublisher<AutoAimSolverSnapshot> statePublisher,
            StringPublisher modeLabelPublisher,
            StructPublisher<Translation3d> fieldTargetPublisher,
            StructPublisher<Translation2d> lookaheadTargetPublisher) {
    }

    private record VisionCameraPublishers(
            StructPublisher<VisionCameraSnapshot> statePublisher,
            StructArrayPublisher<Pose3d> tagPosesPublisher,
            StructPublisher<Pose3d> cameraPosePublisher,
            StructPublisher<Pose3d> robotPosePublisher,
            StructPublisher<Pose2d> relevantPosePublisher) {
    }
}
