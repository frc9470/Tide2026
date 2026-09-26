package com.team9470;

import com.team9470.telemetry.MatchTimingService;
import com.team9470.telemetry.PracticeTimerTracker;
import com.team9470.telemetry.TelemetryManager;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
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
  private final PracticeTimerTracker practiceTimer = new PracticeTimerTracker();
  private final MatchTimingService matchTiming = MatchTimingService.getInstance();
  private final TelemetryManager telemetry = TelemetryManager.getInstance();
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
    MatchType matchType = DriverStation.getMatchType();
    var timing = practiceTimer.update(new PracticeTimerTracker.DriverStationSample(
        Timer.getFPGATimestamp(),
        matchType,
        DriverStation.isFMSAttached(),
        DriverStation.isDSAttached(),
        DriverStation.isAutonomousEnabled(),
        DriverStation.isTeleopEnabled(),
        DriverStation.isTestEnabled(),
        DriverStation.isDisabled(),
        DriverStation.getMatchTime(),
        DriverStation.getAlliance(),
        DriverStation.getGameSpecificMessage()));
    matchTiming.update(timing);
    telemetry.publishPracticeTimerState(timing.snapshot(), timing.phaseLabel(), timing.zoneLabel());
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
