package com.team9470;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

/** Command scheduler, logging, and mode transitions shared by the robot. */
public class Robot extends TimedRobot {
  private static final Path USB_LOG_DIR = Path.of("/u/logs");

  private final RobotContainer robotContainer = new RobotContainer();
  private Command autonomousCommand;

  @Override
  public void robotInit() {
    try {
      if (Files.isDirectory(USB_LOG_DIR.getParent()) && Files.isWritable(USB_LOG_DIR.getParent())) {
        Files.createDirectories(USB_LOG_DIR);
        DataLogManager.start(USB_LOG_DIR.toString());
        DriverStation.startDataLog(DataLogManager.getLog(), false);
      }
    } catch (IOException exception) {
      DriverStation.reportWarning("USB logging unavailable: " + exception.getMessage(), false);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
