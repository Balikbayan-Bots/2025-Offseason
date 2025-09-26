package frc.robot.subsystems.body;

public enum BodySetpoint {
  CORAL_LEVEL1(0.0, 90.0),
  CORAL_LEVEL2(8.5, 60.0),
  CORAL_LEVEL3(17.0, 60.0),
  CORAL_LEVEL4(25.0, 60.0),
  ALGAE_LEVEL2(12.0, 90.0),
  ALGAE_LEVEL3(19.0, 90.0),
  STOW_INTAKE(14.0, 0.0),
  HIGH_NET(26.0, 70.0),
  PROCESSOR(0.0, 0.0),
  SCORE(0.0, 90.0),
  HANDOFF(18, 180);

  private final double elevTravel;
  private final double armDegrees;

  public double getArmDegrees() {
    return armDegrees;
  }

  public double getElevTravel() {
    return elevTravel;
  }

  private BodySetpoint(double elevTravel, double armDegrees) {
    this.elevTravel = elevTravel;

    this.armDegrees = armDegrees;
  }
}
