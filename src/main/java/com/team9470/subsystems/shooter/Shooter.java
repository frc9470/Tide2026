package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.FieldConstants;
import com.team9470.Ports;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.vision.Limelight;
import com.team9470.subsystems.shooter.ShooterConstants;

import edu.wpi.first.math.MathUtil;
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

    // private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX shooterMotor3;
    private final TalonFX shooterMotor4;
    private final TalonFX hoodMotor;

    private final VelocityVoltage motionMagic = new VelocityVoltage(0);
    private final PositionVoltage hoodMotionMagic = new PositionVoltage(0);
    public static double targetShooterVelocity = 0;
    public static Angle targetHoodAngle = Units.Degrees.of(0);
    private double distance = 0;
    private Limelight poseSlave;
    public boolean targetHub = true;

    private boolean homing = true;
    private double homingStallStart = Double.NaN;

    private double
        cachedPivotKP = ShooterConstants.kHoodConfig.Slot0.kP,
        cachedPivotKI = ShooterConstants.kHoodConfig.Slot0.kI,
        cachedPivotKD = ShooterConstants.kHoodConfig.Slot0.kD,
        cachedPivotKS = ShooterConstants.kHoodConfig.Slot0.kS,
        cachedPivotKG = ShooterConstants.kHoodConfig.Slot0.kG,
        cachedPivotKV = ShooterConstants.kHoodConfig.Slot0.kV;

    public Shooter(Limelight swerve) {
        poseSlave = swerve;
        hoodMotor = TalonFXFactory.createDefaultTalon(Ports.HOOD_MOTOR);
        
        //shooterMotor1 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR1); 
        shooterMotor2 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR2); 
        shooterMotor3 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR3); 
        shooterMotor4 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR4); 
        
        // TalonUtil.applyAndCheckConfiguration(shooterMotor1, ShooterConstants.getFrontShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor2, ShooterConstants.getFrontShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor3, ShooterConstants.getBackShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor4, ShooterConstants.getBackShooterConfig());
        TalonUtil.applyAndCheckConfiguration(hoodMotor, ShooterConstants.kHoodConfig);

        
        SmartDashboard.putNumber("Hood/PID/kP", cachedPivotKP);
        SmartDashboard.putNumber("Hood/PID/kI", cachedPivotKI);
        SmartDashboard.putNumber("Hood/PID/kD", cachedPivotKD);
        SmartDashboard.putNumber("Hood/PID/kS", cachedPivotKS);
        SmartDashboard.putNumber("Hood/PID/kV", cachedPivotKV);
        SmartDashboard.putNumber("Hood/PID/kG", cachedPivotKG);
        SmartDashboard.putNumber("Shooter/TargetRpm", targetShooterVelocity);
    }

    @Override
    public void periodic() {
        //homing
        if(homing){
            hoodMotor.setVoltage(ShooterConstants.kHomingVoltage);
            boolean stalled = ShooterConstants.kStallCurrent <= hoodMotor.getStatorCurrent().getValueAsDouble()
                && ShooterConstants.kStallVelocity >= hoodMotor.getVelocity().getValueAsDouble();
            if(stalled){
                double now = Timer.getFPGATimestamp();
                if(Double.isNaN(homingStallStart)){
                    homingStallStart = now;
                }
                else if(stalled && now - homingStallStart > ShooterConstants.kStallTime){
                    hoodMotor.setPosition(ShooterConstants.kHome);
                    hoodMotor.setControl(hoodMotionMagic.withPosition(targetHoodAngle));
                    homing = false;
                }
            }
            else{
                homingStallStart = Double.NaN;
            }
        }

        
        else{
            calculateTargetMain();
        //targetHoodAngle = Units.Rotations.of(MathUtil.clamp(targetHoodAngle.in(Rotations), ShooterConstants.lowerLimit.in(Rotations), ShooterConstants.upperLimit.in(Rotations)));
        //targetVelocity = controller.calculate(getAngle().in(Rotations), targetAngle.in(Rotations));
        //setTargetAngle(targetHoodAngle);
        //setTargetVelocity(targetShooterVelocity);
        // setTargetAngle(calculateHoodAngle());
        // setTargetVelocity(calculateVelocity());
        // shooterMotor1.setControl(motionMagic.withVelocity(targetShooterVelocity));
            shooterMotor2.setControl(motionMagic.withVelocity(targetShooterVelocity));
            shooterMotor3.setControl(motionMagic.withVelocity(targetShooterVelocity));
            shooterMotor4.setControl(motionMagic.withVelocity(targetShooterVelocity));

            hoodMotor.setControl(hoodMotionMagic.withPosition(targetHoodAngle));
        }

        //targetHoodAngle = Degrees.of(SmartDashboard.getNumber("Hood/TargetAngle", 0.0));
        SmartDashboard.putNumber("Shooter/TargetShooterVelocity", targetShooterVelocity);
        SmartDashboard.putNumber("Shooter/ShooterError", getVelocityError());
        
        SmartDashboard.putBoolean("Hood/Homing", homing);
        SmartDashboard.putNumber("Hood/Angle", getAngle().in(Degrees));
        SmartDashboard.putNumber("Hood/Current", hoodMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/TargetAngle", targetHoodAngle.in(Degrees));
        SmartDashboard.putNumber("Hood/Error", getHoodError());
        //targetShooterVelocity = SmartDashboard.getNumber("Shooter/TargetRpm", targetShooterVelocity);
        
        cachedPivotKP = ShooterConstants.kHoodConfig.Slot0.kP;
        cachedPivotKI = ShooterConstants.kHoodConfig.Slot0.kI;
        cachedPivotKD = ShooterConstants.kHoodConfig.Slot0.kD;
        cachedPivotKS = ShooterConstants.kHoodConfig.Slot0.kS;
        cachedPivotKG = ShooterConstants.kHoodConfig.Slot0.kG;
        cachedPivotKV = ShooterConstants.kHoodConfig.Slot0.kV;
        double pivotKP = SmartDashboard.getNumber("Hood/PID/kP", cachedPivotKP);
        double pivotKI = SmartDashboard.getNumber("Hood/PID/kI", cachedPivotKI);
        double pivotKD = SmartDashboard.getNumber("Hood/PID/kD", cachedPivotKD);
        double pivotKG = SmartDashboard.getNumber("Hood/PID/kG", cachedPivotKG);
        double pivotKS = SmartDashboard.getNumber("Hood/PID/kS", cachedPivotKS);
        double pivotKV = SmartDashboard.getNumber("Hood/PID/kV", cachedPivotKV);
        if (pivotKP != cachedPivotKP || pivotKI != cachedPivotKI || pivotKD != cachedPivotKD || pivotKG != cachedPivotKG || pivotKS != cachedPivotKS || pivotKV != cachedPivotKV) {
            cachedPivotKP = pivotKP;
            cachedPivotKI = pivotKI;
            cachedPivotKD = pivotKD;
            cachedPivotKG = pivotKG;
            cachedPivotKS = pivotKS;
            cachedPivotKV = pivotKV;
            Slot0Configs newPivotGains = new Slot0Configs()
                    .withKP(pivotKP).withKI(pivotKI).withKD(pivotKD).withKG(pivotKG).withKS(pivotKS).withKV(pivotKV).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign).withGravityType(GravityTypeValue.Arm_Cosine).withGravityArmPositionOffset(Units.Degrees.of(0));
            hoodMotor.getConfigurator().apply(newPivotGains);
        }
        targetHoodAngle = calculateHoodAngle();
        targetShooterVelocity = (calculateVelocity());
    }

    public Angle getAngle() {
        return hoodMotor.getPosition().getValue();
    }

    public void setTargetAngle(Angle angle) {
        targetHoodAngle = angle;
    }

    public double getVelocity() {
        return shooterMotor3.getVelocity().getValueAsDouble();
    }

    public void setTargetVelocity(double velocity) {
        targetShooterVelocity = velocity;
    }

    public double getVelocityError(){
        return targetShooterVelocity - getVelocity();
    }

    public double getHoodError() {
        return targetHoodAngle.in(Degrees) - getAngle().in(Degrees);
    }

    private void hoodHome(){
        homing = true;
    }

    public Command getHomeHoodCommand(){
        return this.runOnce(
            () -> hoodHome()
        );
    }

    public Command getSetTargetVelocityCommand(double velocity) {
        return this.startEnd(
            () -> setTargetVelocity(velocity), 
            () -> setTargetVelocity(velocity/2)
        );
    }

    private void calculateTargetMain(){
        if(targetHub) calculateTargetHub();
        else calculateTargetPass();
    }

    private Angle calculateHoodAngle(){
        return Degrees.of(4.76151 * distance +5.75854);
    }

    private double calculateVelocity(){
        return 7.0585 * distance+26.26131; 
    }

    private void calculateTargetHub() {
        Pose2d robotPose = poseSlave.getPose2d();

    Translation2d offset = new Translation2d(FieldConstants.Hub.topCenterPoint.getX(),FieldConstants.Hub.topCenterPoint.getY()).minus(robotPose.getTranslation());

    //double fieldAngle = offset.getAngle().getDegrees();
    //double turretAngle = MathUtil.inputModulus( fieldAngle - robotPose.getRotation().getDegrees() - 180, -225, 135);
    SmartDashboard.putNumber("Distance", offset.getNorm());
    distance = offset.getNorm();
    //setTargetAngle(Degrees.of(turretAngle));
        // Pose2d robotPose = poseSlave.getPose();
        // Translation2d offset = new Pose2d(new Translation2d(FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        // Translation2d velocityVector = new Translation2d(Swerve.getInstance().getChassisSpeeds().vxMetersPerSecond,Swerve.getInstance().getChassisSpeeds().vyMetersPerSecond).rotateBy(robotPose.getRotation());
        // velocityVector.times(ShooterConstants.tof);
        // distance = offset.minus(velocityVector).getNorm();
    }

    private void calculateTargetPass() {
        
        // Pose2d robotPose = poseSlave.getPose();
        
        // Translation2d leftPass = new Pose2d(new Translation2d(FieldConstants.Depot.depotCenter.getX(), FieldConstants.Depot.depotCenter.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        // Translation2d rightPass = new Pose2d(new Translation2d(FieldConstants.Outpost.centerPoint.getX(), FieldConstants.Outpost.centerPoint.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        
        // distance = (leftPass.getNorm() < rightPass.getNorm() ? leftPass : rightPass).getNorm();
    }
}
