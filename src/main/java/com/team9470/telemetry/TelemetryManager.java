package com.team9470.telemetry;

import com.team9470.telemetry.structs.DriveStatusSnapshot;
import com.team9470.telemetry.structs.PracticeTimerSnapshot;
import com.team9470.telemetry.structs.VisionCameraSnapshot;
import com.team9470.telemetry.structs.VisionSnapshot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.HashMap;
import java.util.Map;

/** Drive, vision, and match timing telemetry. Add mechanism topics as they are built. */
public final class TelemetryManager {
  public static final int VALIDATION_OK = 0;
  public static final int VALIDATION_OUTSIDE_FIELD = 1;
  public static final int VALIDATION_MAX_VELOCITY = 2;
  public static final int VALIDATION_MAX_CORRECTION = 3;

  private static final TelemetryManager INSTANCE = new TelemetryManager();
  private final NetworkTable root = NetworkTableInstance.getDefault().getTable("Telemetry");
  private final NetworkTable drive = root.getSubTable("Drive");
  private final NetworkTable vision = root.getSubTable("Vision");
  private final StructPublisher<Pose2d> drivePose = drive.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> driveSpeeds = drive.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> moduleStates = drive.getStructArrayTopic("Modules/States", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> moduleTargets = drive.getStructArrayTopic("Modules/Targets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> modulePositions = drive.getStructArrayTopic("Modules/Positions", SwerveModulePosition.struct).publish();
  private final StructPublisher<DriveStatusSnapshot> driveStatus = drive.getStructTopic("Status", DriveStatusSnapshot.struct).publish();
  private final DoubleArrayPublisher moduleVelocity = drive.getDoubleArrayTopic("Modules/DriveVelocityRps").publish();
  private final DoubleArrayPublisher moduleCurrent = drive.getDoubleArrayTopic("Modules/DriveStatorCurrentAmps").publish();
  private final NetworkTable limits = drive.getSubTable("CurrentLimit");
  private final DoublePublisher activeCurrentLimit = limits.getDoubleTopic("ActiveStatorAmps").publish();
  private final DoublePublisher nominalCurrentLimit = limits.getDoubleTopic("NominalStatorAmps").publish();
  private final DoublePublisher turboCurrentLimit = limits.getDoubleTopic("TurboStatorAmps").publish();
  private final BooleanPublisher turboEnabledPublisher = limits.getBooleanTopic("TurboEnabled").publish();
  private final NetworkTable path = drive.getSubTable("Auto");
  private final BooleanPublisher pathActive = path.getBooleanTopic("Active").publish();
  private final DoublePublisher pathSampleTime = path.getDoubleTopic("LastSampleTimestampSec").publish();
  private final StructPublisher<Pose2d> pathDesired = path.getStructTopic("DesiredPose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> pathMeasured = path.getStructTopic("MeasuredPose", Pose2d.struct).publish();
  private final StructPublisher<Translation2d> pathError = path.getStructTopic("PoseError", Translation2d.struct).publish();
  private final DoublePublisher pathHeadingError = path.getDoubleTopic("HeadingErrorRad").publish();
  private final StructPublisher<ChassisSpeeds> pathFeedforward = path.getStructTopic("FeedforwardSpeeds", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> pathFeedback = path.getStructTopic("FeedbackSpeeds", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> pathCommanded = path.getStructTopic("CommandedSpeeds", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> pathMeasuredSpeeds = path.getStructTopic("MeasuredSpeeds", ChassisSpeeds.struct).publish();
  private final DoubleArrayPublisher pathForcesX = path.getDoubleArrayTopic("ModuleForcesX").publish();
  private final DoubleArrayPublisher pathForcesY = path.getDoubleArrayTopic("ModuleForcesY").publish();
  private final DoublePublisher firstPathSample = path.getDoubleTopic("FirstPathSampleSec").publish();
  private final DoublePublisher firstMotionCommand = path.getDoubleTopic("FirstMotionCommandSec").publish();
  private final DoublePublisher centerlineTouch = path.getDoubleTopic("CenterlineTouchSec").publish();
  private final StructPublisher<VisionSnapshot> visionStatus = vision.getStructTopic("Status", VisionSnapshot.struct).publish();
  private final IntegerPublisher visionValidation = vision.getIntegerTopic("ValidationStatus").publish();
  private final NetworkTable timing = root.getSubTable("PracticeTimer");
  private final StructPublisher<PracticeTimerSnapshot> timingStatus = timing.getStructTopic("Status", PracticeTimerSnapshot.struct).publish();
  private final StringPublisher timingPhase = timing.getStringTopic("Phase").publish();
  private final StringPublisher timingZone = timing.getStringTopic("Zone").publish();
  private final Map<String, StructPublisher<VisionCameraSnapshot>> cameraStatus = new HashMap<>();
  private final Map<String, StructArrayPublisher<Pose3d>> cameraTags = new HashMap<>();
  private final Map<String, StructPublisher<Pose3d>> cameraPose = new HashMap<>();
  private final Map<String, StructPublisher<Pose3d>> robotPose = new HashMap<>();
  private final Map<String, StructPublisher<Pose2d>> cameraRelevantPose = new HashMap<>();
  private final Map<String, StructPublisher<Translation2d>> translations = new HashMap<>();
  private final Map<String, StructPublisher<Rotation2d>> rotations = new HashMap<>();
  private final Map<String, StructArrayPublisher<Pose2d>> poses2d = new HashMap<>();
  private final Map<String, StructArrayPublisher<Pose3d>> poses3d = new HashMap<>();
  private int visionValidationStatusCode = VALIDATION_OK;

  private TelemetryManager() {}

  public static TelemetryManager getInstance() { return INSTANCE; }

  public void publishDrivePose(double timestampSec, Pose2d pose) { drivePose.set(pose); }
  public void publishDrivePose(Pose2d pose) { drivePose.set(pose); }
  public void publishDriveSpeeds(double timestampSec, ChassisSpeeds speeds) { driveSpeeds.set(speeds); }
  public void publishDriveSpeeds(ChassisSpeeds speeds) { driveSpeeds.set(speeds); }
  public void publishDriveModuleStates(SwerveModuleState[] states) { moduleStates.set(states); }
  public void publishDriveModuleTargets(SwerveModuleState[] targets) { moduleTargets.set(targets); }
  public void publishDriveModulePositions(SwerveModulePosition[] positions) { modulePositions.set(positions); }
  public void publishDriveStatus(DriveStatusSnapshot snapshot) { driveStatus.set(snapshot); }

  public void publishDriveModuleElectrical(double[] velocityRps, double[] statorCurrentAmps) {
    moduleVelocity.set(velocityRps);
    moduleCurrent.set(statorCurrentAmps);
  }

  public void publishDriveCurrentLimits(double active, double nominal, double turbo, boolean turboEnabled) {
    activeCurrentLimit.set(active);
    nominalCurrentLimit.set(nominal);
    turboCurrentLimit.set(turbo);
    turboEnabledPublisher.set(turboEnabled);
  }

  public void publishDriveAutoPathActive(boolean active) {
    pathActive.set(active);
  }

  public void publishDriveAutoPathSample(double timestampSec, Pose2d desired, Pose2d measured,
      Translation2d translationError, double headingErrorRad, ChassisSpeeds feedforward,
      ChassisSpeeds feedback, ChassisSpeeds commanded, ChassisSpeeds measuredSpeeds,
      double[] moduleForcesX, double[] moduleForcesY) {
    pathSampleTime.set(timestampSec);
    pathDesired.set(desired);
    pathMeasured.set(measured);
    pathError.set(translationError);
    pathHeadingError.set(headingErrorRad);
    pathFeedforward.set(feedforward);
    pathFeedback.set(feedback);
    pathCommanded.set(commanded);
    pathMeasuredSpeeds.set(measuredSpeeds);
    pathForcesX.set(moduleForcesX);
    pathForcesY.set(moduleForcesY);
  }

  public void markDriveAutoFirstPathSample(double timestampSec) {
    firstPathSample.set(timestampSec);
  }

  public void markDriveAutoFirstMotionCommand(double timestampSec, ChassisSpeeds speeds) {
    firstMotionCommand.set(timestampSec);
  }

  public void markDriveAutoCenterlineTouch(double timestampSec, Pose2d pose) {
    centerlineTouch.set(timestampSec);
  }

  public void publishVisionState(VisionSnapshot snapshot) { visionStatus.set(snapshot); }
  public void publishVisionValidationStatusCode(int code) {
    visionValidationStatusCode = code;
    visionValidation.set(code);
  }
  public int getVisionValidationStatusCode() { return visionValidationStatusCode; }

  public void publishPracticeTimerState(PracticeTimerSnapshot snapshot, String phase, String zone) {
    timingStatus.set(snapshot);
    timingPhase.set(phase);
    timingZone.set(zone);
  }

  public void publishVisionCameraState(String name, VisionCameraSnapshot snapshot) {
    cameraStatus.computeIfAbsent(name,
        n -> vision.getSubTable(n).getStructTopic("Status", VisionCameraSnapshot.struct).publish()).set(snapshot);
  }

  public void publishVisionCameraGeometry(String name, Pose3d[] tags, Pose3d camera,
      Pose3d robot, Pose2d relevantPose) {
    NetworkTable table = vision.getSubTable(name);
    cameraTags.computeIfAbsent(name,
        n -> table.getStructArrayTopic("Tags", Pose3d.struct).publish()).set(tags);
    if (camera != null) cameraPose.computeIfAbsent(name,
        n -> table.getStructTopic("CameraPose", Pose3d.struct).publish()).set(camera);
    if (robot != null) robotPose.computeIfAbsent(name,
        n -> table.getStructTopic("RobotPose", Pose3d.struct).publish()).set(robot);
    if (relevantPose != null) cameraRelevantPose.computeIfAbsent(name,
        n -> table.getStructTopic("RelevantPose", Pose2d.struct).publish()).set(relevantPose);
  }

  public void publishTranslation2d(String key, Translation2d translation) {
    translations.computeIfAbsent(key,
        k -> root.getStructTopic("Logs/" + k, Translation2d.struct).publish()).set(translation);
  }

  public void publishRotation2d(String key, Rotation2d rotation) {
    rotations.computeIfAbsent(key,
        k -> root.getStructTopic("Logs/" + k, Rotation2d.struct).publish()).set(rotation);
  }

  public void publishPose2dArray(String key, Pose2d[] poses) {
    poses2d.computeIfAbsent(key,
        k -> root.getStructArrayTopic("Logs/" + k, Pose2d.struct).publish()).set(poses);
  }

  public void publishPose3dArray(String key, Pose3d[] poses) {
    poses3d.computeIfAbsent(key,
        k -> root.getStructArrayTopic("Logs/" + k, Pose3d.struct).publish()).set(poses);
  }
}
