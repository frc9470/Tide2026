package com.team9470.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Ports;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    private final TalonFX rightRollerMotor;
    private final TalonFX leftRollerMotor;
    private final TalonFX rightIntakeMotor;
    private final TalonFX leftIntakeMotor;
    private final TalonFX kickerMotor;

    private final PositionVoltage intakePositionRequest = new PositionVoltage(0);

    private boolean deployed = false;
    private boolean agitating = false;
    private boolean homing = true;
    private boolean outtaking = false;

    private double homingStallStart = Double.NaN;

    private Timer timer = new Timer();

    public Intake() {
        rightRollerMotor = TalonFXFactory.createDefaultTalon(Ports.RIGHT_ROLLER_MOTOR);
        leftRollerMotor = TalonFXFactory.createDefaultTalon(Ports.LEFT_ROLLER_MOTOR);

        rightIntakeMotor = TalonFXFactory.createDefaultTalon(Ports.RIGHT_DEPLOY_MOTOR);
        leftIntakeMotor = TalonFXFactory.createDefaultTalon(Ports.LEFT_DEPLOY_MOTOR);

        kickerMotor = TalonFXFactory.createDefaultTalon(Ports.KICKER_ROLLER);

        TalonUtil.applyAndCheckConfiguration(rightRollerMotor, IntakeConstants.kRightRollerConfig);
        TalonUtil.applyAndCheckConfiguration(leftRollerMotor, IntakeConstants.kLeftRollerConfig);

        TalonUtil.applyAndCheckConfiguration(rightIntakeMotor, IntakeConstants.getConfig());
        TalonUtil.applyAndCheckConfiguration(leftIntakeMotor, IntakeConstants.getReverseConfig());

        TalonUtil.applyAndCheckConfiguration(kickerMotor, IntakeConstants.kKickerRollerConfig);

        leftIntakeMotor.setControl(new Follower(Ports.RIGHT_DEPLOY_MOTOR.getDeviceNumber(), MotorAlignmentValue.Opposed));
        leftRollerMotor.setControl(new Follower(Ports.RIGHT_ROLLER_MOTOR.getDeviceNumber(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        double rollerVolts;
        double kickerVolts;
        double intakeTargetPosition = Double.NaN;

        //homing
        if(homing){
            rightIntakeMotor.setVoltage(IntakeConstants.kHomingVoltage);
            boolean stalled = IntakeConstants.kStallCurrent =< rightIntakeMotor.getStatorCurrent().getValueAsDouble()
                && IntakeConstants.kStallVelocity >= rightIntakeMotor.getVelocity().getValueAsDouble();
            double now = Timer.getFPGATimestamp();
            if(homingStallStart == Double.NaN){
                homingStallStart = now;
            }
            else if(stalled && now - homingStallStart > IntakeConstants.kStallTime){
                rightIntakeMotor.setControl(intakePositionRequest);
                rightIntakeMotor.setPosition(IntakeConstants.kHome);
                homing = false;
            }
            else{
                homingStallStart = Double.NaN;
            }
        }

        //rollers
        if(deployed || agitating){
            if(outtaking) {
                rollerVolts = -IntakeConstants.ROLLER_SPEED;
                kickerVolts = -IntakeConstants.KICKER_SPEED;
            }
            else{
                rollerVolts = IntakeConstants.ROLLER_SPEED;
                kickerVolts = IntakeConstants.KICKER_SPEED;
            }
        }
        else {
            rollerVolts = 0.0;
            kickerVolts = 0.0;
        }

        rightRollerMotor.setVoltage(rollerVolts);
        kickerMotor.setVoltage(kickerVolts);
        
        //lintake
        if(deployed){
            intakeTargetPosition = IntakeConstants.DEPLOYED_POSITION_ROTATIONS;
        }
        else if (agitating) {
            if(timer.get() % 1.0 <= 0.5){
                intakeTargetPosition = IntakeConstants.STOWED_POSITION_ROTATIONS;
            }
            else{
                intakeTargetPosition = IntakeConstants.DEPLOYED_POSITION_ROTATIONS;
            }
        }

        rightIntakeMotor.setControl(intakePositionRequest.withPosition(intakeTargetPosition));

        //Smart Dashboard
        SmartDashboard.putBoolean("Intake/Deployed", deployed);
        SmartDashboard.putBoolean("Intake/Agitating", agitating);
        SmartDashboard.putBoolean("Intake/Homing", homing);

        SmartDashboard.putNumber("Intake/IntakeDeploy/Positions", rightIntakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/IntakeDeploy/TargetRotations", intakeTargetPosition);
        SmartDashboard.putNumber("Intake/IntakeDeploy/PositionError", intakeTargetPosition - rightIntakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/IntakeDeploy/Current/Right Motor", rightIntakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/IntakeDeploy/Current/Left Motor", leftIntakeMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Rollers/Current/Right", rightRollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Current/Left", leftRollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Current/Target", rightIntakeMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Kicker", kickerMotor.getStatorCurrent().getValueAsDouble());
    }

    public void deploy() {
        deployed = true;
        agitating = false;
        homing = false;
    }

    public void stow() {
        deployed = false;
        agitating = false;
        homing = false;
    }

    public void agitate(){
        timer.reset();
        agitating = true;
        homing = false;
        deployed = false;
    }

    public void home(){
        homing = true;
        deployed = false;
        agitating = false;
    }

    public void outtaking(boolean toggle) {
        outtaking = toggle;
    }

    public Command getAgitateCommand() {
        return this.runEnd(
            () -> agitate(),
            () -> deploy()
        );
    }

    public Command getDeployCommand(){
        return this.runOnce(
            () -> deploy()
        );
    }

    public Command getOutakeCommand() {
        return this.runEnd(
            () -> outtaking(true),
            () -> outtaking(false)
        );
    }
    
    //kind of useless, i have no idea why these are here
    public boolean isAgitating() {
        return agitating;
    }

    public boolean isDeployed() {
        return deployed;
        //isDeployed
    }
}