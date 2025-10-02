package frc.robot.vision;

public interface LimelightSubsystem {

  double alignTx();

  double alignTy();

  double alignRz();

  double getTx();

  double getTy();

  boolean getTv();

  void setPipeline(int val);
}
