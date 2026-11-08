package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.FieldConstants;
import com.team9470.Ports;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.turret.TurretConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX shooterMotor3;
    private final TalonFX shooterMotor4;
    private final TalonFX hoodMotor;

    private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodMotionMagic = new MotionMagicVoltage(0);
    public static double targetShooterVelocity = 0;
    public static Angle targetHoodAngle = Units.Degrees.of(0);
    private double distance = 0;
    private Swerve poseSlave;
    public boolean targetHub = true;

    private boolean homing = true;
    private double homingStallStart = Double.NaN;

    public Shooter(Swerve swerve) {
        poseSlave = swerve;
        hoodMotor = TalonFXFactory.createDefaultTalon(Ports.HOOD_MOTOR);
        shooterMotor1 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR1); 
        shooterMotor2 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR2); 
        shooterMotor3 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR3); 
        shooterMotor4 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR4); 
        
        TalonUtil.applyAndCheckConfiguration(shooterMotor1, ShooterConstants.getFrontShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor2, ShooterConstants.getFrontShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor3, ShooterConstants.getBackShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor4, ShooterConstants.getBackShooterConfig());
        TalonUtil.applyAndCheckConfiguration(hoodMotor, ShooterConstants.getHoodConfig());
    }

    @Override
    public void periodic() {


        //homing
        if(homing){
            hoodMotor.setVoltage(ShooterConstants.kHomingVoltage);
            boolean stalled = ShooterConstants.kStallCurrent <= hoodMotor.getStatorCurrent().getValueAsDouble()
                && ShooterConstants.kStallVelocity >= hoodMotor.getVelocity().getValueAsDouble();
            double now = Timer.getFPGATimestamp();
            if(homingStallStart == Double.NaN){
                homingStallStart = now;
            }
            else if(stalled && now - homingStallStart > ShooterConstants.kStallTime){
                hoodMotor.setPosition(ShooterConstants.kHome);
                hoodMotor.setControl(hoodMotionMagic.withPosition(targetHoodAngle));
                homing = false;
            }
            else{
                homingStallStart = Double.NaN;
            }
        }

        calculateTargetMain();
        //targetHoodAngle = Units.Rotations.of(MathUtil.clamp(targetHoodAngle.in(Rotations), ShooterConstants.lowerLimit.in(Rotations), ShooterConstants.upperLimit.in(Rotations)));
        //targetVelocity = controller.calculate(getAngle().in(Rotations), targetAngle.in(Rotations));
        //setTargetAngle(targetHoodAngle);
        //setTargetVelocity(targetShooterVelocity);
        // setTargetAngle(calculateHoodAngle());
        // setTargetVelocity(calculateVelocity());
        shooterMotor1.setControl(motionMagic.withVelocity(targetShooterVelocity));
        shooterMotor2.setControl(motionMagic.withVelocity(targetShooterVelocity));
        shooterMotor3.setControl(motionMagic.withVelocity(targetShooterVelocity));
        shooterMotor4.setControl(motionMagic.withVelocity(targetShooterVelocity));

        hoodMotor.setControl(hoodMotionMagic.withPosition(targetHoodAngle));

        SmartDashboard.putNumber("Shooter/TargetAngle", targetHoodAngle.in(Degrees));
        SmartDashboard.putNumber("Shooter/Error", targetHoodAngle.in(Degrees)-getAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter/TargetShooterVelocity", targetShooterVelocity);
        SmartDashboard.putNumber("Shooter/ShooterError", targetShooterVelocity-getVelocity());
    }

    public Angle getAngle() {
        return hoodMotor.getPosition().getValue();
    }

    public void setTargetAngle(Angle angle) {
        targetHoodAngle = angle;
    }

    public double getVelocity() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    public void setTargetVelocity(double velocity) {
        targetShooterVelocity = velocity;
    }

    private void hoodHome(){
        homing = true;
    }

    public Command getHomeHoodCommand(){
        return this.runOnce(
            () -> hoodHome()
        );
    }

    private void calculateTargetMain(){
        if(targetHub) calculateTargetHub();
        else calculateTargetPass();
    
    }

    @SuppressWarnings("unused")
    private Angle calculateHoodAngle(){
        return Degrees.of(distance); // replace with function
    }

    @SuppressWarnings("unused")
    private double calculateVelocity(){
        return distance; // replace with function
    }

    private void calculateTargetHub() {
        Pose2d robotPose = poseSlave.getPose();
        Translation2d offset = new Pose2d(new Translation2d(FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        Translation2d velocityVector = new Translation2d(Swerve.getInstance().getChassisSpeeds().vxMetersPerSecond,Swerve.getInstance().getChassisSpeeds().vyMetersPerSecond).rotateBy(robotPose.getRotation());
        velocityVector.times(TurretConstants.tof);
        distance = offset.minus(velocityVector).getNorm();
    }

    private void calculateTargetPass() {
        
        Pose2d robotPose = poseSlave.getPose();
        
        Translation2d leftPass = new Pose2d(new Translation2d(FieldConstants.Depot.depotCenter.getX(), FieldConstants.Depot.depotCenter.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        Translation2d rightPass = new Pose2d(new Translation2d(FieldConstants.Outpost.centerPoint.getX(), FieldConstants.Outpost.centerPoint.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        
        distance = (leftPass.getNorm() < rightPass.getNorm() ? leftPass : rightPass).getNorm();
    }
}
