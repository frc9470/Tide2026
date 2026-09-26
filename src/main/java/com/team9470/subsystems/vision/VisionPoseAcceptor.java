package com.team9470.subsystems.vision;

import com.team9470.FieldConstants;
import com.team9470.telemetry.TelemetryManager;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * VisionPoseAcceptor is a class that determines whether a vision update should
 * be accepted or not.
 * 
 * @author frc1678 - 9470 stole code lol
 */
public class VisionPoseAcceptor {
	private static final double FIELD_BORDER_MARGIN = 0.5;
	private static final double MAX_VISION_CORRECTION = 2.0; // Jump from fused pose
	private static final AprilTagFieldLayout aprilTagFieldLayout = FieldConstants.defaultAprilTagType.getLayout();
	private final TelemetryManager telemetry = TelemetryManager.getInstance();

	public boolean shouldAcceptVision(
			double timestamp,
			Pose2d visionFieldToVehicle,
			Pose2d lastFieldToVehicle,
			Twist2d robotVelocity,
			boolean isInAuto) {

		// Check out of field
		if (visionFieldToVehicle.getTranslation().getX() < -FIELD_BORDER_MARGIN
				// || visionFieldToVehicle.getTranslation().getX() > FieldLayout.kFieldLength +
				// FIELD_BORDER_MARGIN
				|| visionFieldToVehicle.getTranslation().getX() > aprilTagFieldLayout.getFieldLength()
						+ FIELD_BORDER_MARGIN
				|| visionFieldToVehicle.getTranslation().getY() < -FIELD_BORDER_MARGIN
				// || visionFieldToVehicle.getTranslation().getY() > FieldLayout.kFieldWidth +
				// FIELD_BORDER_MARGIN) {
				|| visionFieldToVehicle.getTranslation().getY() > aprilTagFieldLayout.getFieldWidth()
						+ FIELD_BORDER_MARGIN) {
			telemetry.publishVisionValidationStatusCode(TelemetryManager.VALIDATION_OUTSIDE_FIELD);
			return false;
		}

		if (Math.hypot(robotVelocity.dx, robotVelocity.dy) > 4.0) {
			telemetry.publishVisionValidationStatusCode(TelemetryManager.VALIDATION_MAX_VELOCITY);
			return false;
		}

		if (isInAuto) {
			// Check max correction
			if (visionFieldToVehicle.getTranslation()
					.getDistance(lastFieldToVehicle.getTranslation()) > MAX_VISION_CORRECTION) {
				telemetry.publishVisionValidationStatusCode(TelemetryManager.VALIDATION_MAX_CORRECTION);
				return false;
			}
		}

		telemetry.publishVisionValidationStatusCode(TelemetryManager.VALIDATION_OK);
		return true;
	}
}
