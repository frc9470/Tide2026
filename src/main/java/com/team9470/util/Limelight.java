package com.team9470.util;

import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    boolean doRejectUpdate;

    public Limelight(){
        
    }

    @Override
    public void periodic(){
        doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", Swerve.getInstance().getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        if(Math.abs(Swerve.getInstance().getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 360)
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            Swerve.getInstance().setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            Swerve.getInstance().addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
    }
    
}
