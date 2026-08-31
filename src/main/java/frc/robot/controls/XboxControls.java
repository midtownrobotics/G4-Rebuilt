package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

/** G4 Xbox mapping based on midtownrobotics/G3-2026. */
public class XboxControls implements Controls {
  private final CommandXboxController controller;
  private double rumbleValue;

  public XboxControls(int controllerPort) {
    controller = new CommandXboxController(controllerPort);
  }

  @Override
  public double getDriveForward() {
    return -MathUtil.applyDeadband(controller.getLeftY(), kDriverJoystickThreshold);
  }

  @Override
  public double getDriveLeft() {
    return -MathUtil.applyDeadband(controller.getLeftX(), kDriverJoystickThreshold);
  }

  @Override
  public double getDriveRotation() {
    boolean keyboardJoystick = RobotBase.isSimulation() && DriverStation.getStickAxisCount(0) <= 3;
    double axis = keyboardJoystick ? controller.getHID().getRawAxis(2) : controller.getRightX();
    return -MathUtil.applyDeadband(axis, kDriverJoystickThreshold);
  }

  @Override
  public Trigger idle() {
    return controller.leftBumper();
  }

  @Override
  public Trigger intake() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger shoot() {
    return controller.rightBumper().and(disableShooting().negate());
  }

  @Override
  public Trigger snowBlow() {
    return controller.rightTrigger();
  }

  @Override
  public Trigger unjam() {
    return controller.y();
  }

  @Override
  public Trigger feedFuel() {
    return controller.b().and(zeroHood().negate());
  }

  @Override
  public Trigger setpointShoot() {
    return controller.a();
  }

  @Override
  public Trigger setpointFeed() {
    return controller.x().and(zeroIntake().negate());
  }

  @Override
  public Trigger zeroIntake() {
    return controller.leftBumper().and(controller.x());
  }

  @Override
  public Trigger zeroHood() {
    return controller.leftBumper().and(controller.b());
  }

  @Override
  public Trigger disableShooting() {
    return controller.leftBumper().and(controller.rightBumper());
  }

  @Override
  public Trigger increaseHoodAngle() {
    return controller.povUp();
  }

  @Override
  public Trigger decreaseHoodAngle() {
    return controller.povDown();
  }

  private void setRumble(boolean enabled) {
    double nextValue = enabled ? 0.5 : 0.0;
    if (nextValue != rumbleValue) {
      controller.getHID().setRumble(RumbleType.kBothRumble, nextValue);
      rumbleValue = nextValue;
    }
  }

  @Override
  public Command rumbleCommand() {
    return Commands.run(() -> setRumble(true)).finallyDo(() -> setRumble(false));
  }

  @Override
  public Command pulseRumbleCommand(int pulses, double pulseDuration) {
    List<Command> commands = new ArrayList<>();
    for (int i = 0; i < pulses; i++) {
      commands.add(rumbleCommand().withTimeout(pulseDuration));
      if (i < pulses - 1) {
        commands.add(Commands.waitSeconds(0.1));
      }
    }
    return Commands.sequence(commands.toArray(Command[]::new));
  }
}
