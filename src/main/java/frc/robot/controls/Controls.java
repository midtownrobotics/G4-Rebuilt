package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Driver inputs, kept separate from the robot mechanisms for easy controller remapping. */
public interface Controls {
  double kDriverJoystickThreshold = 0.1;

  double getDriveForward();

  double getDriveLeft();

  double getDriveRotation();

  Trigger idle();

  Trigger intake();

  Trigger shoot();

  Trigger snowBlow();

  Trigger unjam();

  Trigger feedFuel();

  Trigger setpointShoot();

  Trigger setpointFeed();

  Trigger zeroIntake();

  Trigger zeroHood();

  Trigger disableShooting();

  Trigger increaseHoodAngle();

  Trigger decreaseHoodAngle();

  Command rumbleCommand();

  Command pulseRumbleCommand(int pulses, double pulseDuration);
}
