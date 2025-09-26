package frc.robot.subsystems.manipulators;

import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_CENTER_MOTOR_ID;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_DEPLOY_MOTOR_ID;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_FEED_FWD;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_GEAR_RATIO;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_MAX_VOLTAGE_FWD;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_MAX_VOLTAGE_REVERSE;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_MOTION_MAGIC_CONFIGS;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_ROLLERS_MOTOR_ID;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.INTAKE_SLOT_ZERO;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.kIntakeLimits;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsytem extends SubsystemBase {
  public static IntakeSubsytem m_instance;

  public static IntakeSubsytem getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSubsytem();
    }
    return m_instance;
  }

  private TalonFX deployMotor;
  private TalonFX centerMotor;
  private TalonFX rollersMotor;

  private IntakeState state = IntakeState.START;

  private double refrenceDegrees = 0;

  private MotionMagicVoltage motionMagic;

  private IntakeSetpoint activeSetpoint = IntakeSetpoint.STOWED_HANDOFF;

  private IntakeSubsytem() {

    deployMotor = new TalonFX(INTAKE_DEPLOY_MOTOR_ID);
    centerMotor = new TalonFX(INTAKE_CENTER_MOTOR_ID);
    rollersMotor = new TalonFX(INTAKE_ROLLERS_MOTOR_ID);

    reZero();

    motionMagic = new MotionMagicVoltage(0).withSlot(0);
    BaseStatusSignal.setUpdateFrequencyForAll(200, deployMotor.getPosition());
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        deployMotor.getSupplyVoltage(),
        deployMotor.getFault_Hardware(),
        deployMotor.getMotorVoltage(),
        deployMotor.getSupplyCurrent(),
        deployMotor.getStatorCurrent(),
        deployMotor.getFault_DeviceTemp());
    deployMotor.optimizeBusUtilization();
  }

  private void configureMotor(TalonFXConfigurator motorConfig) {
    TalonFXConfiguration newConfig = new TalonFXConfiguration();

    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = false; // TODO: PUT ACTUAL LIMITS
    limits.ReverseSoftLimitEnable = false;

    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kIntakeLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kIntakeLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = INTAKE_MAX_VOLTAGE_FWD; // out
    voltage.PeakReverseVoltage = INTAKE_MAX_VOLTAGE_REVERSE; // in

    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = INTAKE_SLOT_ZERO[0];
    slot0.kI = INTAKE_SLOT_ZERO[1];
    slot0.kD = INTAKE_SLOT_ZERO[2];
    slot0.kS = INTAKE_SLOT_ZERO[3];
    slot0.kG = INTAKE_SLOT_ZERO[4];
    slot0.kV = INTAKE_SLOT_ZERO[5];
    slot0.kA = INTAKE_SLOT_ZERO[6];

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    var output = newConfig.MotorOutput;
    output.NeutralMode = NeutralModeValue.Brake;
    motionMagic.MotionMagicAcceleration = INTAKE_MOTION_MAGIC_CONFIGS[0];
    motionMagic.MotionMagicCruiseVelocity = INTAKE_MOTION_MAGIC_CONFIGS[1];
    motionMagic.MotionMagicJerk = INTAKE_MOTION_MAGIC_CONFIGS[2];
    motorConfig.apply(newConfig);
  }

  public void reZero() {
    deployMotor.setPosition(0);
  }

  public void updateReference(double degrees) {
    refrenceDegrees = degrees;
  }

  public void periodic() {
    updateReference(activeSetpoint.getDegrees());
    deployMotor.setControl(
        motionMagic
            .withPosition(degreesToMotorRotations(refrenceDegrees))
            .withSlot(0)
            .withFeedForward(calculateFeedForward()));
    rollersMotor.set(state.getRollerMotorSpeed());
    centerMotor.set(state.getCenterMotorSpeed());
  }

  private double calculateFeedForward() {
    return Math.sin(Math.toRadians(getDegrees() + 18)) * -(INTAKE_FEED_FWD);
  }

  private double getDegrees() {
    return motorRotationsToDegrees(deployMotor.getPosition().getValueAsDouble());
  }

  public static double motorRotationsToDegrees(double rotations) {
    return (rotations / INTAKE_GEAR_RATIO) * 360;
  }

  public static double degreesToMotorRotations(double degrees) {
    return (degrees * INTAKE_GEAR_RATIO) / 360;
  }

  public double getReferenceDegrees() {
    return refrenceDegrees;
  }

  public void updateSetpoint(IntakeSetpoint setPoint) {
    activeSetpoint = setPoint;
  }

  public void setState(IntakeState state) {
    this.state = state;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Intake Degrees", this::getDegrees, null);
    builder.addDoubleProperty("Intake Feed Forward", this::calculateFeedForward, null);
  }
}
