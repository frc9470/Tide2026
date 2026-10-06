package com.team9470.subsystems.dye;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Ports;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dye extends SubsystemBase {
    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private double targetVelocityRps = 0;

    public Dye() {
        motor = TalonFXFactory.createDefaultTalon(Ports.DYE_MOTOR);
        TalonUtil.applyAndCheckConfiguration(motor, DyeConstants.getConfig());
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("DyeRotor/VelocityRps", getVelocityRps());
        SmartDashboard.putNumber("DyeRotor/TargetRps", targetVelocityRps);
        SmartDashboard.putNumber("DyeRotor/StatorCurrentAmps", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("DyeRotor/AtSpeed", isAtSpeed());
    }

    public double getVelocityRps() {
        return motor.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed() {
        return targetVelocityRps != 0
                && MathUtil.isNear(targetVelocityRps, getVelocityRps(), DyeConstants.VELOCITY_TOLERANCE_RPS);
    }

    private void setVelocity(double rps) {
        targetVelocityRps = rps;
        motor.setControl(velocityRequest.withVelocity(rps));
    }

    private void setNeutral() {
        targetVelocityRps = 0;
        motor.setControl(neutralRequest);
    }

    public Command feed() {
        return runEnd(() -> setVelocity(DyeConstants.FEED_VELOCITY_RPS), this::setNeutral);
    }

    public Command reverse() {
        return runEnd(() -> setVelocity(DyeConstants.REVERSE_VELOCITY_RPS), this::setNeutral);
    }

    public Command stop() {
        return run(this::setNeutral);
    }
}
