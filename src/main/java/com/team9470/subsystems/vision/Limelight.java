package com.team9470.subsystems.vision;

import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    boolean doRejectUpdate;
    public Pose2d pose;

    public Limelight(){
        
    }

    @Override
    public void periodic(){
        doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", Swerve.getInstance().getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        SmartDashboard.putNumber("Pose/Translation/x", mt2.pose.getX() );
        SmartDashboard.putNumber("Pose/Translation/y", mt2.pose.getY() );
        SmartDashboard.putNumber("Pose/Translation/Angle", mt2.pose.getRotation().getDegrees() );
        
        // if(Math.abs(Swerve.getInstance().getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 360)
        // {
        //     doRejectUpdate = true;
        // }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
         if(!doRejectUpdate)
         {
            this.pose = mt2.pose;
            SmartDashboard.putNumber("Pose/timestamp", mt2.timestampSeconds );
            Swerve.getInstance().setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
            Swerve.getInstance().addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds, VecBuilder.fill(0.3,0.3,9999999));
        }
    }
    public Pose2d getPose2d(){
        return pose;
    }
}
