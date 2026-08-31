package frc.lib;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.constants.Constants;

/** A G3-style AdvantageKit number that is editable under {@code /Tuning} in tuning mode. */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String TABLE_KEY = "/Tuning";

  private final String key;
  private final double defaultValue;
  private final LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastValues = new HashMap<>();

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    key = TABLE_KEY + "/" + dashboardKey;
    this.defaultValue = defaultValue;
    dashboardNumber = Constants.kTuningMode ? new LoggedNetworkNumber(key, defaultValue) : null;
  }

  public double get() {
    return Constants.kTuningMode ? dashboardNumber.get() : defaultValue;
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastValues.put(id, currentValue);
    return lastValue == null || currentValue != lastValue;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(number -> number.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
