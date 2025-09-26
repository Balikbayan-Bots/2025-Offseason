package frc.robot.subsystems.manipulators;

import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.BodyConstants;
import frc.robot.subsystems.body.BodyConstants.Limits;

public class ManipulatorConstants {
  public static final double algae_speed = -0.5;
  public static final int CLAW_MOTOR_ID = 31;
  public static final int INTAKE_DEPLOY_MOTOR_ID = 40;
  public static final int INTAKE_CENTER_MOTOR_ID = 41;
  public static final int INTAKE_ROLLERS_MOTOR_ID = 42;
  public static final double INTAKE_GEAR_RATIO = 17.778D;
  public static final int BEAM_BREAK_ID = 9;
  public static final int CLAW_MAX_VOLTAGE_FORWARD = 10;
  public static final int CLAW_MAX_VOLTAGE_REVERSE = -10;
  public static BodyConstants.Limits kclawLimits = new Limits(85.0, 65.0, 0, 0);

  public static final Limits kIntakeLimits =
      new Limits(
          70.0,
          35.0,
          ArmSubsystem.degreesToMotorRotations(0),
          ArmSubsystem.degreesToMotorRotations(-133));

  public static final double INTAKE_MAX_VOLTAGE_FWD = 8;
  public static final double INTAKE_MAX_VOLTAGE_REVERSE = -8;

  public static final double[] INTAKE_SLOT_ZERO = {
    0.0, // kP
    0.0, // kI
    0.0, // kD
    0.0, // kS
    0.0, // kG
    0.01, // kV
    0.0 // kA
  };

  public static final double INTAKE_FEED_FWD = 0.001;

  public static final double[] INTAKE_MOTION_MAGIC_CONFIGS = {
    40.0, // Acceleration
    20.0, // Cruise Velocity
    1800.0 // Jerk
  };
}
