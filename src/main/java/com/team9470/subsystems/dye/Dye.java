package com.team9470.subsystems.dye;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Ports;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dye extends SubsystemBase {
    private final TalonFX dye;
    private final TalonFX leftRoller;
    private final TalonFX rightRoller;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private double targetVelocityRps = 0;
    private double rollerTargetRps = 0;

    public Dye() {
        dye = TalonFXFactory.createDefaultTalon(Ports.DYE_MOTOR);
        leftRoller = TalonFXFactory.createDefaultTalon(Ports.DYE_LEFT_ROLLER);
        rightRoller = TalonFXFactory.createDefaultTalon(Ports.DYE_RIGHT_ROLLER);
        TalonUtil.applyAndCheckConfiguration(dye, DyeConstants.getConfig());
        TalonUtil.applyAndCheckConfiguration(leftRoller, DyeConstants.getRollerConfig());
        TalonUtil.applyAndCheckConfiguration(rightRoller, DyeConstants.getRollerFollowerConfig());
        rightRoller.setControl(new Follower(Ports.DYE_LEFT_ROLLER.getDeviceNumber(),
                DyeConstants.RIGHT_ROLLER_OPPOSES_LEFT ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("DyeRotor/VelocityRps", getVelocityRps());
        SmartDashboard.putNumber("DyeRotor/TargetRps", targetVelocityRps);
        SmartDashboard.putNumber("DyeRotor/StatorCurrentAmps", dye.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("DyeRotor/AtSpeed", isAtSpeed());
        SmartDashboard.putNumber("DyeRollers/VelocityRps", getRollerVelocityRps());
        SmartDashboard.putNumber("DyeRollers/TargetRps", rollerTargetRps);
        SmartDashboard.putNumber("DyeRollers/LeftStatorCurrentAmps", leftRoller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("DyeRollers/RightStatorCurrentAmps",
                rightRoller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("DyeRollers/AtSpeed", isRollersAtSpeed());
    }

    public double getVelocityRps() {
        return dye.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed() {
        return targetVelocityRps != 0
                && MathUtil.isNear(targetVelocityRps, getVelocityRps(), DyeConstants.VELOCITY_TOLERANCE_RPS);
    }

    public double getRollerVelocityRps() {
        return leftRoller.getVelocity().getValueAsDouble();
    }

    public boolean isRollersAtSpeed() {
        return rollerTargetRps != 0
                && MathUtil.isNear(rollerTargetRps, getRollerVelocityRps(), DyeConstants.ROLLER_VELOCITY_TOLERANCE_RPS);
    }

    private void setVelocity(double rotorRps, double rollerRps) {
        targetVelocityRps = rotorRps;
        rollerTargetRps = rollerRps;
        dye.setControl(velocityRequest.withVelocity(rotorRps));
        leftRoller.setControl(rollerRequest.withVelocity(rollerRps));
    }

    private void setNeutral() {
        targetVelocityRps = 0;
        rollerTargetRps = 0;
        dye.setControl(neutralRequest);
        leftRoller.setControl(neutralRequest);
    }

    public Command feed() {
        return runEnd(() -> setVelocity(DyeConstants.FEED_VELOCITY_RPS, DyeConstants.ROLLER_FEED_VELOCITY_RPS),
                this::setNeutral);
    }

    public Command reverse() {
        return runEnd(() -> setVelocity(DyeConstants.REVERSE_VELOCITY_RPS, DyeConstants.ROLLER_REVERSE_VELOCITY_RPS),
                this::setNeutral);
    }

    public Command stop() {
        return run(this::setNeutral);
    }
}
