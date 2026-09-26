package com.team9470.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;

/**
 * Constants describing camera placement relative to the robot frame.
 */
public final class VisionConstants {
        private VisionConstants() {
        }

        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(
                        Units.Inches.of(+13.220055), // x
                        Units.Inches.of(10.073682), // y
                        Units.Inches.of(+18.904), // z
                        new Rotation3d(
                                        0, // roll
                                        Math.toRadians(-19.657357), // pitch
                                        Math.toRadians(-40) // yaw
                        ));

        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(
                        Units.Inches.of(+13.220055), // x
                        Units.Inches.of(-10.073682), // y
                        Units.Inches.of(+18.904), // z
                        new Rotation3d(
                                        0, // roll
                                        Math.toRadians(-19.657357), // pitch
                                        Math.toRadians(40) // yaw
                        ));
}
