package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX shooterMotor3;
    private final TalonFX shooterMotor4;
    private final TalonFX hoodMotor;
    private double cachedHoodKP, cachedHoodKI, cachedHoodKD, cachedHoodKV, cachedHoodKG;
    private double cachedShooterKP, cachedShooterKI, cachedShooterKD, cachedShooterKV, cachedShooterKS;

    private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodMotionMagic = new MotionMagicVoltage(0);
    public static double targetShooterVelocity = 0;
    public static double targetHoodAngle = 0;
    private double distance = 0;
    private Swerve poseSlave;
    public boolean targetHub = true;
    
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
        initSmartDashboardGains();
    }

    @Override
    public void periodic() {
        updateTargets();
        updateSmartDashboardGains();
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
        SmartDashboard.putNumber("shooter/TargetAngle", targetHoodAngle);
        SmartDashboard.putNumber("shooter/Error", targetHoodAngle-getAngle().in(Degrees));
        SmartDashboard.putNumber("shooter/TargetShooterVelocity", targetShooterVelocity);
        SmartDashboard.putNumber("shooter/ShooterError", targetShooterVelocity-getVelocity());
    }
    public Angle getAngle() {
        return hoodMotor.getPosition().getValue();
    }

    public void setTargetAngle(double angle) {
        targetHoodAngle = angle;
    }

    public double getVelocity() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    public void setTargetVelocity(double velocity) {
        targetShooterVelocity = velocity;
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
    private void updateTargets(){
        targetHoodAngle = SmartDashboard.getNumber("shooter/TargetAngle", targetHoodAngle);
        targetShooterVelocity = SmartDashboard.getNumber("shooter/TargetShooterVelocity", targetShooterVelocity);
    }
    private void initSmartDashboardGains() {
        cachedHoodKP = ShooterConstants.getHoodConfig().Slot0.kP;
        cachedHoodKD = ShooterConstants.getHoodConfig().Slot0.kD;
        cachedHoodKV = ShooterConstants.getHoodConfig().Slot0.kV;
        cachedHoodKG = ShooterConstants.getHoodConfig().Slot0.kG;

        SmartDashboard.putNumber("Hood/kP", cachedHoodKP);
        SmartDashboard.putNumber("Hood/kD", cachedHoodKD);
        SmartDashboard.putNumber("Hood/kV", cachedHoodKV);
        SmartDashboard.putNumber("Hood/kG", cachedHoodKG);

        cachedShooterKP = ShooterConstants.kP;
        cachedShooterKD = ShooterConstants.kD;
        cachedShooterKV = ShooterConstants.kV;
        cachedShooterKS = ShooterConstants.kS;

        SmartDashboard.putNumber("Shooter/kP", cachedShooterKP);
        SmartDashboard.putNumber("Shooter/kS", cachedShooterKS);
        SmartDashboard.putNumber("Shooter/kD", cachedShooterKD);
        SmartDashboard.putNumber("Shooter/kV", cachedShooterKV);
    }

    private void updateSmartDashboardGains() {
        double hoodKP = SmartDashboard.getNumber("Hood/kP", cachedHoodKP);
        double hoodKD = SmartDashboard.getNumber("Hood/kD", cachedHoodKD);
        double hoodKV = SmartDashboard.getNumber("Hood/kV", cachedHoodKV);
        double hoodKG = SmartDashboard.getNumber("Hood/kG", cachedHoodKG);

        if (hoodKP != cachedHoodKP  || hoodKD != cachedHoodKD
                || hoodKV != cachedHoodKV || hoodKG != cachedHoodKG) {
            cachedHoodKP = hoodKP;
            cachedHoodKD = hoodKD;
            cachedHoodKV = hoodKV;
            cachedHoodKG = hoodKG;

            Slot0Configs hoodGains = new Slot0Configs()
                    .withKP(hoodKP)
                    .withKD(hoodKD)
                    .withKV(hoodKV)
                    .withKG(hoodKG);
            hoodMotor.getConfigurator().apply(hoodGains);
        }

        double shooterKP = SmartDashboard.getNumber("Shooter/kP", cachedShooterKP);
        double shooterKD = SmartDashboard.getNumber("Shooter/kD", cachedShooterKD);
        double shooterKV = SmartDashboard.getNumber("Shooter/kV", cachedShooterKV);
        double shooterKS = SmartDashboard.getNumber("Shooter/kS", cachedShooterKS);

        if (shooterKP != cachedShooterKP  || shooterKD != cachedShooterKD
                || shooterKV != cachedShooterKV || shooterKS != cachedShooterKS) {
            cachedShooterKP = shooterKP;
            cachedShooterKD = shooterKD;
            cachedShooterKV = shooterKV;
            cachedShooterKS = shooterKS;

            Slot0Configs hoodGains = new Slot0Configs()
                    .withKP(shooterKP)
                    .withKD(shooterKD)
                    .withKV(shooterKV)
                    .withKS(shooterKS);
            shooterMotor1.getConfigurator().apply(hoodGains);
            shooterMotor2.getConfigurator().apply(hoodGains);
            shooterMotor3.getConfigurator().apply(hoodGains);
            shooterMotor4.getConfigurator().apply(hoodGains);
        }
    }
}
