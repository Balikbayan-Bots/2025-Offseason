package frc.robot.subsystems.manipulators;

public enum ClawState {
  INTAKE(-0.8), // Example speed
  OUTAKE(0.6), // Example speed
  HOLDING_ALGAE(0.5),
  IDLE(0.0),
  SCORE(-0.45);

  public final double speed;

  private ClawState(double speed) {
    this.speed = speed;
  }

  public double getSpeed() {
    return this.speed;
  }
}
