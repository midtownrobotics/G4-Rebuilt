package frc.lib;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Logs command activity, interruptions, and subsystem ownership for AdvantageScope. */
public final class LoggedCommandScheduler {
  private static final String LOG_KEY = "Commands";
  private static final String ALERT_TYPE = "Alerts";

  private static final Set<Command> runningCommands = new HashSet<>();
  private static final Map<Command, Command> runningInterrupters = new HashMap<>();
  private static final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

  private LoggedCommandScheduler() {}

  public static void init(CommandScheduler scheduler) {
    scheduler.onCommandInitialize(LoggedCommandScheduler::commandStarted);
    scheduler.onCommandFinish(LoggedCommandScheduler::commandEnded);
    scheduler.onCommandInterrupt((interrupted, interrupting) -> {
      interrupting.ifPresent(interrupter -> runningInterrupters.put(interrupter, interrupted));
      commandEnded(interrupted);
    });
  }

  private static void commandStarted(Command command) {
    if (!runningInterrupters.containsKey(command)) {
      runningCommands.add(command);
    }
    for (Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.put(subsystem, command);
    }
  }

  private static void commandEnded(Command command) {
    runningCommands.remove(command);
    runningInterrupters.remove(command);
    for (Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.remove(subsystem);
    }
  }

  public static void periodic() {
    Logger.recordOutput(LOG_KEY + "/Running/.type", ALERT_TYPE);
    Logger.recordOutput(
        LOG_KEY + "/Running/warnings",
        runningCommands.stream().map(Command::getName).toArray(String[]::new));

    Logger.recordOutput(LOG_KEY + "/Running/errors", runningInterrupters.entrySet().stream()
        .map(entry -> interruptionDescription(entry.getKey(), entry.getValue()))
        .toArray(String[]::new));

    Logger.recordOutput(LOG_KEY + "/Subsystems/.type", ALERT_TYPE);
    Logger.recordOutput(LOG_KEY + "/Subsystems/infos", requiredSubsystems.entrySet().stream()
        .map(entry -> entry.getKey().getName() + " (" + entry.getValue().getName() + ")")
        .toArray(String[]::new));
  }

  private static String interruptionDescription(Command interrupter, Command interrupted) {
    Set<Subsystem> commonRequirements = new HashSet<>(interrupter.getRequirements());
    commonRequirements.retainAll(interrupted.getRequirements());
    String requirements = String.join(",", commonRequirements.stream().map(Subsystem::getName).toList());
    return interrupter.getName() + " interrupted " + interrupted.getName() + " (" + requirements + ")";
  }
}
