// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Adapted for Team 9470 Alpha2026
// Based on 6328's FieldConstants pattern for the 2026 season.

package com.team9470;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

/**
 * Contains field geometry constants for the 2026 FRC game.
 * All constants are defined relative to the field coordinate system,
 * and from the perspective of the blue alliance station.
 * 
 * <p>
 * Use {@link com.team9470.util.AllianceFlipUtil} to flip geometry for red
 * alliance.
 */
public class FieldConstants {

        // Field type selection - change this to match your practice field
        public static final FieldType fieldType = FieldType.WELDED;

        // AprilTag related constants
        public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
        public static final double aprilTagWidth = Units.inchesToMeters(6.5);
        public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

        // Field dimensions (from AprilTag layout)
        public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
        public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

        /**
         * Safely get a tag pose with a fallback default position.
         * Prevents crashes when using fallback layouts that lack certain tag IDs.
         */
        private static Pose3d getTagPoseOrDefault(int tagId, Pose3d defaultPose) {
                Optional<Pose3d> pose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(tagId);
                return pose.orElse(defaultPose);
        }

        // Default poses for when tags are missing (centered on field at reasonable
        // positions)
        private static final Pose3d DEFAULT_HUB_NEAR_POSE = new Pose3d(
                        Units.inchesToMeters(120), fieldWidth / 2.0, Units.inchesToMeters(36), new Rotation3d());
        private static final Pose3d DEFAULT_HUB_FAR_POSE = new Pose3d(
                        fieldLength - Units.inchesToMeters(120), fieldWidth / 2.0, Units.inchesToMeters(36),
                        new Rotation3d());
        private static final Pose3d DEFAULT_TOWER_POSE = new Pose3d(
                        Units.inchesToMeters(43.51), fieldWidth / 2.0, Units.inchesToMeters(45), new Rotation3d());
        private static final Pose3d DEFAULT_OPP_TOWER_POSE = new Pose3d(
                        fieldLength - Units.inchesToMeters(43.51), fieldWidth / 2.0, Units.inchesToMeters(45),
                        new Rotation3d());
        private static final Pose3d DEFAULT_OUTPOST_POSE = new Pose3d(
                        0, fieldWidth / 2.0, Units.inchesToMeters(28), new Rotation3d());

        /**
         * Officially defined vertical lines on the field (X-axis offsets)
         */
        public static class LinesVertical {
                public static final double center = fieldLength / 2.0;
                public static final double starting = getTagPoseOrDefault(26, DEFAULT_HUB_NEAR_POSE).getX();
                public static final double allianceZone = starting;
                public static final double hubCenter = getTagPoseOrDefault(26, DEFAULT_HUB_NEAR_POSE).getX()
                                + Hub.width / 2.0;
                public static final double neutralZoneNear = center - Units.inchesToMeters(120);
                public static final double neutralZoneFar = center + Units.inchesToMeters(120);
                public static final double oppHubCenter = getTagPoseOrDefault(4, DEFAULT_HUB_FAR_POSE).getX()
                                + Hub.width / 2.0;
                public static final double oppAllianceZone = getTagPoseOrDefault(10, DEFAULT_HUB_FAR_POSE).getX();
        }

        /**
         * Officially defined horizontal lines on the field (Y-axis offsets)
         */
        public static class LinesHorizontal {
                public static final double center = fieldWidth / 2.0;
                // Right of hub
                public static final double rightBumpStart = Hub.nearRightCorner.getY();
                public static final double rightBumpEnd = rightBumpStart - RightBump.width;
                public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
                public static final double rightTrenchOpenEnd = 0;
                // Left of hub
                public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
                public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
                public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
                public static final double leftTrenchOpenStart = fieldWidth;
        }

        /**
         * Hub (central scoring goal) related constants
         */
        public static class Hub {
                // Dimensions
                public static final double width = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top
                public static final double innerWidth = Units.inchesToMeters(41.7);
                public static final double innerHeight = Units.inchesToMeters(56.5);

                // Relevant reference points on alliance side
                public static final Translation3d topCenterPoint = new Translation3d(
                                getTagPoseOrDefault(26, DEFAULT_HUB_NEAR_POSE).getX() + width / 2.0,
                                fieldWidth / 2.0,
                                height);
                public static final Translation3d innerCenterPoint = new Translation3d(
                                getTagPoseOrDefault(26, DEFAULT_HUB_NEAR_POSE).getX() + width / 2.0,
                                fieldWidth / 2.0,
                                innerHeight);
                public static final Translation2d nearLeftCorner = new Translation2d(
                                topCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d nearRightCorner = new Translation2d(
                                topCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);
                public static final Translation2d farLeftCorner = new Translation2d(
                                topCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d farRightCorner = new Translation2d(
                                topCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);

                // Relevant reference points on the opposite side
                public static final Translation3d oppTopCenterPoint = new Translation3d(
                                getTagPoseOrDefault(4, DEFAULT_HUB_FAR_POSE).getX() + width / 2.0,
                                fieldWidth / 2.0,
                                height);
                public static final Translation2d oppNearLeftCorner = new Translation2d(
                                oppTopCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d oppNearRightCorner = new Translation2d(
                                oppTopCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);
                public static final Translation2d oppFarLeftCorner = new Translation2d(
                                oppTopCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d oppFarRightCorner = new Translation2d(
                                oppTopCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);

                // Hub faces (AprilTag poses)
                public static final Pose2d nearFace = getTagPoseOrDefault(26, DEFAULT_HUB_NEAR_POSE).toPose2d();
                public static final Pose2d farFace = getTagPoseOrDefault(20, DEFAULT_HUB_FAR_POSE).toPose2d();
                public static final Pose2d rightFace = getTagPoseOrDefault(18, DEFAULT_HUB_NEAR_POSE).toPose2d();
                public static final Pose2d leftFace = getTagPoseOrDefault(21, DEFAULT_HUB_NEAR_POSE).toPose2d();
        }

        /**
         * Left Bump related constants
         */
        public static class LeftBump {
                public static final double width = Units.inchesToMeters(73.0);
                public static final double height = Units.inchesToMeters(6.513);
                public static final double depth = Units.inchesToMeters(44.4);

                public static final Translation2d nearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
                public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
                public static final Translation2d farLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
                public static final Translation2d farRightCorner = Hub.farLeftCorner;
        }

        /**
         * Right Bump related constants
         */
        public static class RightBump {
                public static final double width = Units.inchesToMeters(73.0);
                public static final double height = Units.inchesToMeters(6.513);
                public static final double depth = Units.inchesToMeters(44.4);

                public static final Translation2d nearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
                public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
                public static final Translation2d farLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
                public static final Translation2d farRightCorner = Hub.farLeftCorner;
        }

        /**
         * Left Trench related constants
         */
        public static class LeftTrench {
                public static final double width = Units.inchesToMeters(65.65);
                public static final double depth = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(40.25);
                public static final double openingWidth = Units.inchesToMeters(50.34);
                public static final double openingHeight = Units.inchesToMeters(22.25);

                public static final Translation3d openingTopLeft = new Translation3d(
                                LinesVertical.hubCenter, fieldWidth, openingHeight);
                public static final Translation3d openingTopRight = new Translation3d(
                                LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);
                public static final Translation3d oppOpeningTopLeft = new Translation3d(
                                LinesVertical.oppHubCenter, fieldWidth, openingHeight);
                public static final Translation3d oppOpeningTopRight = new Translation3d(
                                LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
        }

        /**
         * Right Trench related constants
         */
        public static class RightTrench {
                public static final double width = Units.inchesToMeters(65.65);
                public static final double depth = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(40.25);
                public static final double openingWidth = Units.inchesToMeters(50.34);
                public static final double openingHeight = Units.inchesToMeters(22.25);

                public static final Translation3d openingTopLeft = new Translation3d(
                                LinesVertical.hubCenter, openingWidth, openingHeight);
                public static final Translation3d openingTopRight = new Translation3d(
                                LinesVertical.hubCenter, 0, openingHeight);
                public static final Translation3d oppOpeningTopLeft = new Translation3d(
                                LinesVertical.oppHubCenter, openingWidth, openingHeight);
                public static final Translation3d oppOpeningTopRight = new Translation3d(
                                LinesVertical.oppHubCenter, 0, openingHeight);
        }

        /**
         * Tower (climbing structure) related constants
         */
        public static class Tower {
                public static final double width = Units.inchesToMeters(49.25);
                public static final double depth = Units.inchesToMeters(45.0);
                public static final double height = Units.inchesToMeters(78.25);
                public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
                public static final double frontFaceX = Units.inchesToMeters(43.51);
                public static final double uprightHeight = Units.inchesToMeters(72.1);

                // Rung heights from the floor
                public static final double lowRungHeight = Units.inchesToMeters(27.0);
                public static final double midRungHeight = Units.inchesToMeters(45.0);
                public static final double highRungHeight = Units.inchesToMeters(63.0);

                // Relevant reference points on alliance side
                public static final Translation2d centerPoint = new Translation2d(
                                frontFaceX,
                                getTagPoseOrDefault(31, DEFAULT_TOWER_POSE).getY());
                public static final Translation2d leftUpright = new Translation2d(
                                frontFaceX,
                                getTagPoseOrDefault(31, DEFAULT_TOWER_POSE).getY()
                                                + innerOpeningWidth / 2 + Units.inchesToMeters(0.75));
                public static final Translation2d rightUpright = new Translation2d(
                                frontFaceX,
                                getTagPoseOrDefault(31, DEFAULT_TOWER_POSE).getY()
                                                - innerOpeningWidth / 2 - Units.inchesToMeters(0.75));

                // Relevant reference points on opposing side
                public static final Translation2d oppCenterPoint = new Translation2d(
                                fieldLength - frontFaceX,
                                getTagPoseOrDefault(15, DEFAULT_OPP_TOWER_POSE).getY());
                public static final Translation2d oppLeftUpright = new Translation2d(
                                fieldLength - frontFaceX,
                                getTagPoseOrDefault(15, DEFAULT_OPP_TOWER_POSE).getY()
                                                + innerOpeningWidth / 2 + Units.inchesToMeters(0.75));
                public static final Translation2d oppRightUpright = new Translation2d(
                                fieldLength - frontFaceX,
                                getTagPoseOrDefault(15, DEFAULT_OPP_TOWER_POSE).getY()
                                                - innerOpeningWidth / 2 - Units.inchesToMeters(0.75));
        }

        /**
         * Depot (game piece loading) related constants
         */
        public static class Depot {
                public static final double width = Units.inchesToMeters(42.0);
                public static final double depth = Units.inchesToMeters(27.0);
                public static final double height = Units.inchesToMeters(1.125);
                public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

                public static final Translation3d depotCenter = new Translation3d(
                                depth, (fieldWidth / 2) + distanceFromCenterY, height);
                public static final Translation3d leftCorner = new Translation3d(
                                depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
                public static final Translation3d rightCorner = new Translation3d(
                                depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
        }

        /**
         * Outpost related constants
         */
        public static class Outpost {
                public static final double width = Units.inchesToMeters(31.8);
                public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
                public static final double height = Units.inchesToMeters(7.0);

                public static final Translation2d centerPoint = new Translation2d(
                                0, getTagPoseOrDefault(29, DEFAULT_OUTPOST_POSE).getY());
        }

        /**
         * Game piece constants
         */
        public static class GamePiece {
                public static final double ballRadius = Units.inchesToMeters(2.75);
                public static final double ballDiameter = ballRadius * 2.0;
        }

        /**
         * Field type enumeration for different field layouts
         */
        public enum FieldType {
                ANDYMARK("andymark"),
                WELDED("welded");

                private final String jsonFolder;

                FieldType(String jsonFolder) {
                        this.jsonFolder = jsonFolder;
                }

                public String getJsonFolder() {
                        return jsonFolder;
                }
        }

        /**
         * AprilTag layout types with lazy loading
         */
        public enum AprilTagLayoutType {
                OFFICIAL("2026-official"),
                NONE("2026-none");

                private final String name;
                private volatile AprilTagFieldLayout layout;
                private volatile String layoutString;

                AprilTagLayoutType(String name) {
                        this.name = name;
                }

                public AprilTagFieldLayout getLayout() {
                        if (layout == null) {
                                synchronized (this) {
                                        if (layout == null) {
                                                try {
                                                        Path p;
                                                        if (Robot.isSimulation()) {
                                                                // In simulation, load from source directory
                                                                p = Path.of(
                                                                                "src", "main", "deploy", "apriltags",
                                                                                fieldType.getJsonFolder(),
                                                                                name + ".json");
                                                        } else {
                                                                // On robot, load from deploy directory
                                                                p = Path.of(
                                                                                Filesystem.getDeployDirectory()
                                                                                                .getPath(),
                                                                                "apriltags", fieldType.getJsonFolder(),
                                                                                name + ".json");
                                                        }
                                                        layout = new AprilTagFieldLayout(p);
                                                        layoutString = new ObjectMapper().writeValueAsString(layout);
                                                } catch (IOException e) {
                                                        // Fallback to WPILib default layout
                                                        System.err.println("Failed to load AprilTag layout: "
                                                                        + e.getMessage());
                                                        System.err.println(
                                                                        "Falling back to WPILib default 2025 layout");
                                                        layout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
                                                                        edu.wpi.first.apriltag.AprilTagFields.k2025ReefscapeWelded);
                                                        try {
                                                                layoutString = new ObjectMapper()
                                                                                .writeValueAsString(layout);
                                                        } catch (IOException e2) {
                                                                layoutString = "{}";
                                                        }
                                                }
                                        }
                                }
                        }
                        return layout;
                }

                public String getLayoutString() {
                        if (layoutString == null) {
                                getLayout();
                        }
                        return layoutString;
                }
        }
}
