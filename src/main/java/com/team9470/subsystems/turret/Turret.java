package com.team9470.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.FieldConstants;
import com.team9470.Ports;
import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;

    private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);
    private PIDController controller = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
    private double targetVelocity = 0;
    private Angle targetAngle = Units.Degrees.of(0);
    private Swerve poseSlave;
    public Turret(Swerve swerve) {
        poseSlave = swerve;
        turretMotor = TalonFXFactory.createDefaultTalon(Ports.TURRET_MOTOR); 
        TalonUtil.applyAndCheckConfiguration(turretMotor, TurretConstants.getTurretConfig());
    }

    @Override
    public void periodic() {
        calculateTarget();
        targetVelocity = controller.calculate(getAngle().in(Rotations), targetAngle.in(Rotations));
        turretMotor.setControl(motionMagic.withVelocity(targetVelocity));
        //no logging haha
    }
    public Angle getAngle() {
        return turretMotor.getPosition().getValue();
    }

    private void setTargetAngle(Angle angle) {
        targetAngle = angle;
    }

    private void calculateTarget() {
        Pose2d robotPose = poseSlave.getPose();
        Translation2d offset = new Pose2d(new Translation2d(FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY()), new Rotation2d()).minus(robotPose).getTranslation();
        setTargetAngle(offset.getAngle().getMeasure());
    }

}
