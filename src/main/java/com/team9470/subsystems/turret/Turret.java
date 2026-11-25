package com.team9470.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.FieldConstants;
import com.team9470.Ports;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.vision.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
public class Turret extends SubsystemBase {

    private static TalonFX turretMotor;

    private final PositionVoltage motionMagic = new PositionVoltage(0);
    //private PIDController controller = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
    //private double targetVelocity = 0;
    public static Angle targetAngle = Units.Degrees.of(0);
    private Limelight poseSlave;
    public boolean targetHub = true;

    private double
        cachedPivotKP = TurretConstants.kTurretConfig.Slot0.kP,
        cachedPivotKI = TurretConstants.kTurretConfig.Slot0.kI,
        cachedPivotKD = TurretConstants.kTurretConfig.Slot0.kD,
        cachedPivotKS = TurretConstants.kTurretConfig.Slot0.kS,
        cachedPivotKG = TurretConstants.kTurretConfig.Slot0.kG,
        cachedPivotKV = TurretConstants.kTurretConfig.Slot0.kV;
    
    public Turret(Limelight swerve) {
        poseSlave = swerve;
        turretMotor = TalonFXFactory.createDefaultTalon(Ports.TURRET_MOTOR); 
        TalonUtil.applyAndCheckConfiguration(turretMotor, TurretConstants.kTurretConfig);

        SmartDashboard.putNumber("Turret/PID/kP", cachedPivotKP);
        SmartDashboard.putNumber("Turret/PID/kI", cachedPivotKI);
        SmartDashboard.putNumber("Turret/PID/kD", cachedPivotKD);
        SmartDashboard.putNumber("Turret/PID/kS", cachedPivotKS);
        SmartDashboard.putNumber("Turret/PID/kV", cachedPivotKV);
        SmartDashboard.putNumber("Turret/PID/kG", cachedPivotKG);
        
    }
    
    @Override
    public void periodic() {
        calculateTargetHub();
        //targetAngle = Units.Degrees.of(MathUtil.clamp(targetAngle.in(Degrees), TurretConstants.lowerLimit.in(Degrees), TurretConstants.upperLimit.in(Degrees)));
        
        boolean withinTolerance = Math.abs(getAngle().minus(targetAngle).in(Degree)) <= TurretConstants.kTolerance;
        // if(!withinTolerance){
            turretMotor.setControl(motionMagic.withPosition(targetAngle));
        // }
        // else{
        //     turretMotor.setPosition(targetAngle);
        // }
        SmartDashboard.putNumber("Turret/TargetAngle", targetAngle.in(Degrees));
        SmartDashboard.putNumber("Turret/Angle", getAngle().in(Degrees));        
        SmartDashboard.putNumber("Turret/Error", targetAngle.in(Degrees)-getAngle().in(Degrees));
        
        targetAngle = Degrees.of(SmartDashboard.getNumber("Turret/TargetAngle", targetAngle.in(Degrees)));

        cachedPivotKP = TurretConstants.kTurretConfig.Slot0.kP;
        cachedPivotKI = TurretConstants.kTurretConfig.Slot0.kI;
        cachedPivotKD = TurretConstants.kTurretConfig.Slot0.kD;
        cachedPivotKS = TurretConstants.kTurretConfig.Slot0.kS;
        cachedPivotKG = TurretConstants.kTurretConfig.Slot0.kG;
        cachedPivotKV = TurretConstants.kTurretConfig.Slot0.kV;
        double pivotKP = SmartDashboard.getNumber("Turret/PID/kP", cachedPivotKP);
        double pivotKI = SmartDashboard.getNumber("Turret/PID/kI", cachedPivotKI);
        double pivotKD = SmartDashboard.getNumber("Turret/PID/kD", cachedPivotKD);
        double pivotKG = SmartDashboard.getNumber("Turret/PID/kG", cachedPivotKG);
        double pivotKS = SmartDashboard.getNumber("Turret/PID/kS", cachedPivotKS);
        double pivotKV = SmartDashboard.getNumber("Turret/PID/kV", cachedPivotKV);
        if (pivotKP != cachedPivotKP || pivotKI != cachedPivotKI || pivotKD != cachedPivotKD || pivotKG != cachedPivotKG || pivotKS != cachedPivotKS || pivotKV != cachedPivotKV) {
            cachedPivotKP = pivotKP;
            cachedPivotKI = pivotKI;
            cachedPivotKD = pivotKD;
            cachedPivotKG = pivotKG;
            cachedPivotKS = pivotKS;
            cachedPivotKV = pivotKV;
            Slot0Configs newPivotGains = new Slot0Configs()
                    .withKP(pivotKP).withKI(pivotKI).withKD(pivotKD).withKG(pivotKG).withKS(pivotKS).withKV(pivotKV).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign).withGravityType(GravityTypeValue.Arm_Cosine).withGravityArmPositionOffset(Units.Degrees.of(-45));
            turretMotor.getConfigurator().apply(newPivotGains);
        }
    }

    public static Angle getAngle() {
        return turretMotor.getPosition().getValue();
    }

    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
    }

    public Command getSetTargetAngleCommand(Angle angle){
        return Commands.runOnce(() -> setTargetAngle(angle));
    }

    public Command getTurretAngleCommand(){
        return Commands.runOnce(() -> getAngle());
    }

    // public void getAimCommand(){

    //     setTargetAngle();
    // }

    // @SuppressWarnings("unused")
    // private void calculateTargetMain(){
    //     if(targetHub) calculateTargetHub();
    //     else calculateTargetPass();
    
    // }
private void calculateTargetHub() {
    Pose2d robotPose = poseSlave.getPose2d();

    Translation2d offset = new Translation2d(FieldConstants.Hub.topCenterPoint.getX(),FieldConstants.Hub.topCenterPoint.getY()).minus(robotPose.getTranslation());

    double fieldAngle = offset.getAngle().getDegrees();
    double turretAngle = MathUtil.inputModulus( fieldAngle - robotPose.getRotation().getDegrees() - 180, -225, 135);
    SmartDashboard.putNumber("Distance", offset.getNorm());
    setTargetAngle(Degrees.of(turretAngle));
}
    

    public static Angle calculateTarget() {
        Pose2d robotPose = Swerve.getInstance().getPose();
        Translation2d offset = new Pose2d(new Translation2d(FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        Translation2d velocityVector = new Translation2d(Swerve.getInstance().getChassisSpeeds().vxMetersPerSecond,Swerve.getInstance().getChassisSpeeds().vyMetersPerSecond).rotateBy(robotPose.getRotation());
        velocityVector.times(TurretConstants.tof);
        return offset.minus(velocityVector).getAngle().getMeasure().plus(Radians.of(Math.PI));
    }
    // private void calculateTargetPass() {
        
    //     Pose2d robotPose = poseSlave.getPose();
        
    //     Translation2d leftPass = new Pose2d(new Translation2d(FieldConstants.Depot.depotCenter.getX(), FieldConstants.Depot.depotCenter.getY()), new Rotation2d()).minus(robotPose).getTranslation();
    //     Translation2d rightPass = new Pose2d(new Translation2d(FieldConstants.Outpost.centerPoint.getX(), FieldConstants.Outpost.centerPoint.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        
    //     setTargetAngle((leftPass.getNorm() < rightPass.getNorm() ? leftPass : rightPass).getAngle().getMeasure());
    // }

}
