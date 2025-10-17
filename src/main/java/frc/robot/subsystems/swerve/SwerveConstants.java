package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveConstants {
  public static final LinearVelocity SPEED_AT_12V = MetersPerSecond.of(5.84);
  public static final Double SLOW_SPEED = 0.5;
  public static final Double MAX_TELEOP_SPEED = 0.35 * SPEED_AT_12V.in(MetersPerSecond);
  public static final Double MAX_TELEOP_ROT =
      0.35 * RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  public static final Module FRONT_LEFT_MODULE = new Module(Rotations.of(0.39892578125));
  public static final Module FRONT_RIGHT_MODULE = new Module(Rotations.of(0.067626953125));
  public static final Module BACK_LEFT_MODULE = new Module(Rotations.of(-0.041015625));
  public static final Module BACK_RIGHT_MODULE = new Module(Rotations.of(0.294921875));

  public static final Current SLIP_CURRENT = Amps.of(80.0);
  public static final Current STATOR_CURRENT = Amps.of(60.0);

  public static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(50)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(1.4)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  public static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs().withKP(0.08229).withKI(0).withKD(0).withKS(0.1).withKV(0.124);

  public record Module(Angle encoderOffset) {}
}
