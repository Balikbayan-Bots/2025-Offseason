package frc.robot.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLimelightSubsystem extends SubsystemBase implements LimelightSubsystem {

  private static IntakeLimelightSubsystem m_instance;

  public static IntakeLimelightSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeLimelightSubsystem();
    }
    return m_instance;
  }

  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-cbotint");
  private double x;
  private double y;
  private double rZ;
  private boolean Tv;

  private double x_kP = 0.07D;
  private double x_kD = 0.0;
  private double x_targetAlgea = -12.57;
  private double x_output = 0.0;
  private double x_outputAlgea = 0.0;
  private PIDController tx_controller = new PIDController(x_kP, 0.0, x_kD);

  private double rZ_kP = 0.6D;
  private double y_kP = 0.055D;
  private double y_kD = 0.0D;
  private double y_targetAlgea = -1.42;
  private double y_output = 0.0;
  private double y_outputAlgea = 0.0;
  private PIDController ty_controller = new PIDController(y_kP, 0.0, y_kD);

  private double limelightMountingAngleDegrees = 31.475;
  private double limelightMountZ = 30.345;
  private double limelightMountX = 8.589;
  private double limelihgtMountY = 3.325;
  private double krot = 0.2;

  public enum AimingState {
    IDLE,
    ACTIVE_AIM,
    COASTING
  }

  public AimingState currentAimingState = AimingState.IDLE;

  public double lastForwardcommand = 0;
  public double lastRotationCommand = 0;

  public static final double kTargetTy = 0.0;
  public static final double kTyP = 0.5;
  public static final double kStoppingTyError = 0.1;

  public double alignTx() {
    return x_output;
  }

  public double alignTy() {
    return -y_output;
  }

  public double alignTxAlgea() {
    return x_outputAlgea;
  }

  public double alignTyAlgea() {
    return -y_outputAlgea;
  }

  public double alignRz() {
    double output = x * krot;
    return output;
  }

  public double getTx() {
    return x;
  }

  public double getTy() {
    return y;
  }

  private double getRz() {

    return -x * krot;
    // return rZ;
  }

  public boolean getTv() {
    return Tv;
  }

  public void setPipeline(int pipeline) {
    limelight.getEntry("pipeline").setNumber(pipeline);
  }

  public int getPipeline() {
    return (int) limelight.getEntry("pipeline").getDouble(0);
  }

  @Override
  public void periodic() {
    x = limelight.getEntry("tx").getDouble(0);
    limelight.getEntry("ta").getDouble(0);
    y = limelight.getEntry("ty").getDouble(0);
    Tv = limelight.getEntry("tv").getDouble(0) == 1;
    rZ =
        LimelightHelpers.pose3dToArray(
            LimelightHelpers.getTargetPose3d_CameraSpace("limelight-Intake"))[4];

    y_output = ty_controller.calculate(y, 0.0);

    x_output = tx_controller.calculate(x, 0.0);

    y_outputAlgea = ty_controller.calculate(y, y_targetAlgea);

    x_outputAlgea = tx_controller.calculate(x, x_targetAlgea);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake LL");

    builder.addDoubleProperty("Limelight Ty Output", this::alignTy, null);
    builder.addDoubleProperty("Limelight Tx Output", this::alignTx, null);
    builder.addDoubleProperty("Limelight Rz Output", this::alignRz, null);
    builder.addDoubleProperty("Limelight Tx", this::getTx, null);
    builder.addDoubleProperty("Limelight Ty", this::getTy, null);
    builder.addDoubleProperty("Limelight Rz", this::getRz, null);
    builder.addDoubleProperty("Limelight Ty P", this::getKp_y, this::setKp_y);

    // Pipeline
    builder.addIntegerProperty("limelight pipeline", this::getPipeline, null);
  }

  public double getKp_y() {
    return y_kP;
  }

  public void setKp_y(double newValue) {
    y_kP = newValue;
  }
}
