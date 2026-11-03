package com.team9470.subsystems.vision;

import com.team9470.FieldConstants;
import com.team9470.Robot;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.VisionSnapshot;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.turret.Turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.List;

import static com.team9470.subsystems.vision.VisionConstants.FRONT_LEFT_CAMERA_OFFSET;
import static com.team9470.subsystems.vision.VisionConstants.FRONT_RIGHT_CAMERA_OFFSET;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private final VisionDevice turret = new VisionDevice("OV2311-Turret", FRONT_LEFT_CAMERA_OFFSET);
    private final VisionDevice side = new VisionDevice("OV2311-Side", FRONT_RIGHT_CAMERA_OFFSET);
    // private final VisionDevice back = new VisionDevice("back",
    // VisionConstants.BACK_CAMERA_OFFSET);
    private boolean visionDisabled = true; // Enable after camera names and transforms are verified.

    private final List<VisionDevice> devices = List.of(turret, side);
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    // ----------------- SIMULATION -----------------
    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;
    private VisionSystemSim visionSim;

    public Vision() {
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(FieldConstants.defaultAprilTagType.getLayout());
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            var leftCameraProp = new SimCameraProperties();
            leftCameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(90));
            leftCameraProp.setCalibError(0.4, 0.10);
            leftCameraProp.setFPS(25);
            leftCameraProp.setAvgLatencyMs(50);
            leftCameraProp.setLatencyStdDevMs(15);

            var rightCameraProp = new SimCameraProperties();
            rightCameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(95.38));
            rightCameraProp.setCalibError(1.1, 0.10);
            rightCameraProp.setFPS(25);
            rightCameraProp.setAvgLatencyMs(50);
            rightCameraProp.setLatencyStdDevMs(15);

            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            leftCameraSim = new PhotonCameraSim(turret.returnCam(), leftCameraProp);
            rightCameraSim = new PhotonCameraSim(side.returnCam(), rightCameraProp);

            leftCameraSim.enableDrawWireframe(true);
            rightCameraSim.enableDrawWireframe(true);

            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(leftCameraSim, FRONT_LEFT_CAMERA_OFFSET);
            visionSim.addCamera(rightCameraSim, FRONT_RIGHT_CAMERA_OFFSET);

        }
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    int count = 0;

    @Override
    public void periodic() {
        Transform3d transform = new Transform3d(FRONT_LEFT_CAMERA_OFFSET.getTranslation(), new Rotation3d(0,0,Turret.getAngle().in(Radians)));
        turret.updateOffset(transform);
        devices.forEach(visionDevice -> visionDevice.updatePosition(Swerve.getInstance()));
        int connectedCount = (int) devices.stream().filter(VisionDevice::isConnected).count();
        telemetry.publishVisionState(new VisionSnapshot(
                count,
                connectedCount == devices.size(),
                visionDisabled,
                connectedCount,
                devices.size(),
                telemetry.getVisionValidationStatusCode()));
        count++;
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(Swerve.getInstance().getPose());
    }

    public boolean isFullyConnected() {
        return devices.stream().allMatch(VisionDevice::isConnected);
    }

    public boolean isVisionDisabled() {
        return visionDisabled;
    }

    public void setVisionDisabled(boolean visionDisabled) {
        this.visionDisabled = visionDisabled;
    }
}
