package com.team9470;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import choreo.auto.AutoFactory;
import com.team9470.commands.TideAutoRoutine;
import com.team9470.commands.WheelRadiusCharacterization;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.vision.Limelight;
import com.team9470.subsystems.vision.Vision;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.team9470.subsystems.turret.Turret;
import com.team9470.subsystems.dye.Dye;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

/** Shared drive controls and places to add the clone's mechanisms and autonomous routines. */
public class RobotContainer {
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Swerve swerve;
  private Intake intake;
  //privae Turret turret;
  private Shooter shooter;
  //private Limelight limelight;
  private Dye dye;

  public RobotContainer() {
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("AutoChooser", autoChooser);

    // TunerConstants contains Alpha2026 IDs, offsets, and dimensions. Verify them on
    // the clone before enabling construction of motor controllers.
    if (!HardwareConfig.DRIVETRAIN_VERIFIED) {
      DriverStation.reportWarning("Drivetrain disabled: verify TunerConstants and HardwareConfig", false);
      swerve = null;
      return;
    }
    intake = new Intake();
    
    //limelight = new Limelight();
    swerve = Swerve.getInstance();
    shooter = new Shooter(swerve);
    //turret = new Turret(swerve);
    dye = new Dye();

    if (HardwareConfig.VISION_VERIFIED) {
      Vision.getInstance().setVisionDisabled(false);
    }
    configureDriveBindings();
    configureAutos();
  }

  private void configureAutos() {
    //AutoFactory autoFactory = swerve.createAutoFactory();
    //autoChooser.addOption("Tide Auto", TideAutoRoutine.build(autoFactory, swerve, intake, dye));
  }

  private void configureDriveBindings() {
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double maxAngularRate = Math.toRadians(TunerConstants.maxAngularVelocity);
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();

    swerve.setDefaultCommand(swerve.applyRequest(() -> drive
        .withVelocityX(-driver.getLeftY() * maxSpeed)
        .withVelocityY(-driver.getLeftX() * maxSpeed)
        .withRotationalRate(driver.getRightX() * maxAngularRate )));
    driver.x().whileTrue(swerve.applyRequest(() -> xLock));
    driver.back().and(DriverStation::isTestEnabled)
        .whileTrue(new WheelRadiusCharacterization(swerve));

      
//Dye
dye.reverse();
new Trigger(() -> driver.getRightTriggerAxis() > 0.3)
    .whileTrue(new RunCommand(() -> dye.feed(), dye))
    .onFalse(new InstantCommand(() -> dye.reverse(), dye));
intake.deploy();
// Intake
new Trigger(() -> driver.getLeftTriggerAxis() > 0.3)
    .whileTrue(new RunCommand(() -> intake.agitate(), intake))
    .onFalse(new InstantCommand(() -> intake.deploy(), intake));
    
// //Shooter
shooter.setTargetVelocity(150);
//shooter.setTargetAngle(Units.Rotations.of(30));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
