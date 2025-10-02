package frc.robot.subsystems.body;

public enum BodySetpoint {
  START_CONFIG(0.0, 0.0),
  SAFE_START(18.0, 90),
  // CORAL_LEVEL1(0.0, 90.0),
  CORAL_LEVEL2(2.0, 45.0),
  CORAL_LEVEL3(10.0, 45.0),
  CORAL_LEVEL4(24.0, 55.0),
  ALGAE_LEVEL2(12.0, 90.0),
  ALGAE_LEVEL3(19.0, 90.0),
  SAFE_STOW(20.0, 90.0),
  STOW_POS(20.0, 180.0),
  HIGH_NET(26.0, 70.0),
  PROCESSOR(0.0, 0.0),
  SCORE(0.0, 80.0),
  HANDOFF(18, 180),
  SCORE_LEVEL2(0.0, 50.0);

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
