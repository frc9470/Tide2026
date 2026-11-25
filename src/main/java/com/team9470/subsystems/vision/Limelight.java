package com.team9470.subsystems.vision;

import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    boolean doRejectUpdate;
    public Pose2d pose = new Pose2d();

    public Limelight(){
        
    }

    @Override
    public void periodic(){
        doRejectUpdate = false;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            pose = mt1.pose;
        SmartDashboard.putNumber("Pose/x", mt1.pose.getX());
        SmartDashboard.putNumber("Pose/y", mt1.pose.getY());
        SmartDashboard.putNumber("Pose/Heading", pose.getRotation().getRadians());
        SmartDashboard.putNumber("Pose/time", mt1.timestampSeconds);
            Swerve.getInstance().setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            Swerve.getInstance().addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }
    }
    public Pose2d getPose2d(){
        return pose;
    }
}
