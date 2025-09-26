package frc.robot.subsystems.manipulators;

public enum IntakeSetpoint {
  STOWED_HANDOFF(0.0),
  DEPLOYED(-133.0),
  LVL_ONE(-35.0);

  private final double degrees;

  public double getDegrees() {
    return degrees;
  }

  private IntakeSetpoint(double degrees) {
    this.degrees = degrees;
  }
}
