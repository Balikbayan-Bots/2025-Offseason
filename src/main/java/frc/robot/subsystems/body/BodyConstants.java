package frc.robot.subsystems.body;

public class BodyConstants {
  public static final int ELEV_MOTOR_LEFT = 20;
  public static final int ELEV_MOTOR_RIGHT = 21;
  public static final int ELEV_MAGNET_ID = -1;

  public static final int ARM_MOTOR_ID = 30;
  public static final int ARM_MAGNET_ID = -1;

  public static final double ELEV_SPROCKET_CIRCUMFERENCE = 1.273 * Math.PI;

  public static final double ARM_GEAR_RATIO = 32D / 1D;

  public static final double ELEV_GEAR_RATIO = 6.25D / 1D;

  public static final double ARM_MAX_VOLTAGE_FWD = 14;
  public static final double ARM_MAX_VOLTAGE_REVERSE = -14;

  public static final double ELEV_MAX_VOLTAGE_FWD = 12;
  public static final double ELEV_MAX_VOLATGE_REVERSE = -4;
  public static final double ELEV_MAX_POS = 27;

  public static final double[] ELEV_SLOT_ZERO = {
    2.0, // kP
    0.0, // kI
    0.0, // kD
    0.0, // kS
    0.0, // kG
    0.105, // kV
    0.0 // kA
  };

  public static final double ELEV_FEED_FWD = 0.5;

  public static final double[] ARM_SLOT_ZERO = {
    1.75, // kP
    0.0, // kI
    0.0, // kD
    0.0, // kS
    0.0, // kG
    0.1, // kV
    0.0 // kA
  };

  public static final double ARM_FEED_FWD = 1.0;

  public static final double[] ELEV_MOTION_MAGIC_CONFIGS = {
    90.0, // Acceleration
    20.0, // Cruise Velocity
    1600.0 // Jerk
  };

  public static final double[] ARM_MOTION_MAGIC_CONFIGS = {
    40.0, // Acceleration
    20.0, // Cruise Velocity
    1800.0 // Jerk
  };

  public static final Limits kArmLimits =
      new Limits(
          90.0,
          45.0,
          ArmSubsystem.degreesToMotorRotations(180),
          ArmSubsystem.degreesToMotorRotations(-45));

  public static final Limits kElevLimits =
      new Limits(70.0, 45.0, ElevatorSubsystem.inchesToMotorRotations(27), -0.05);

  public record Limits(
      double statorLimit, double supplyLimit, double forwardLimit, double reverseLimit) {}
  ;
}
