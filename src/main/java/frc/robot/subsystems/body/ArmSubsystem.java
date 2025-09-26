package frc.robot.subsystems.body;

import static frc.robot.subsystems.body.BodyConstants.ARM_FEED_FWD;
import static frc.robot.subsystems.body.BodyConstants.ARM_GEAR_RATIO;
import static frc.robot.subsystems.body.BodyConstants.ARM_MAX_VOLTAGE_FWD;
import static frc.robot.subsystems.body.BodyConstants.ARM_MAX_VOLTAGE_REVERSE;
import static frc.robot.subsystems.body.BodyConstants.ARM_MOTION_MAGIC_CONFIGS;
import static frc.robot.subsystems.body.BodyConstants.ARM_MOTOR_ID;
import static frc.robot.subsystems.body.BodyConstants.ARM_SLOT_ZERO;
import static frc.robot.subsystems.body.BodyConstants.kArmLimits;

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

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem m_instance;

  public static ArmSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ArmSubsystem();
    }
    return m_instance;
  }

  private TalonFX motor;
  private BodySetpoint activeSetpoint = BodySetpoint.STOW_INTAKE;
  private double refrenceDegrees = 0;
  private MotionMagicVoltage motionMagic;

  private ArmSubsystem() {
    motor = new TalonFX(ARM_MOTOR_ID);
    configureMotor(motor.getConfigurator());
    reZero();
    motionMagic = new MotionMagicVoltage(0).withSlot(0);
    BaseStatusSignal.setUpdateFrequencyForAll(200, motor.getPosition());
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor.getSupplyVoltage(),
        motor.getFault_Hardware(),
        motor.getMotorVoltage(),
        motor.getSupplyCurrent(),
        motor.getStatorCurrent(),
        motor.getFault_DeviceTemp());

    motor.optimizeBusUtilization();
  }

  public void updateSetpoint(BodySetpoint setPoint) {
    activeSetpoint = setPoint;
  }

  private void configureMotor(TalonFXConfigurator motorConfig) {
    TalonFXConfiguration newConfig = new TalonFXConfiguration();

    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = false; // TODO: PUT ACTUAL LIMITS
    limits.ReverseSoftLimitEnable = false;

    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kArmLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kArmLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = ARM_MAX_VOLTAGE_FWD; // out
    voltage.PeakReverseVoltage = ARM_MAX_VOLTAGE_REVERSE; // in

    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = ARM_SLOT_ZERO[0];
    slot0.kI = ARM_SLOT_ZERO[1];
    slot0.kD = ARM_SLOT_ZERO[2];
    slot0.kS = ARM_SLOT_ZERO[3];
    slot0.kG = ARM_SLOT_ZERO[4];
    slot0.kV = ARM_SLOT_ZERO[5];
    slot0.kA = ARM_SLOT_ZERO[6];

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    var output = newConfig.MotorOutput;
    output.NeutralMode = NeutralModeValue.Brake;
    motionMagic.MotionMagicAcceleration = ARM_MOTION_MAGIC_CONFIGS[0];
    motionMagic.MotionMagicCruiseVelocity = ARM_MOTION_MAGIC_CONFIGS[1];
    motionMagic.MotionMagicJerk = ARM_MOTION_MAGIC_CONFIGS[2];
    motorConfig.apply(newConfig);
  }

  public void reZero() {
    motor.setPosition(0);
  }

  public void updateReference(double degrees) {
    refrenceDegrees = degrees;
  }

  @Override
  public void periodic() {
    updateReference(activeSetpoint.getArmDegrees());
    motor.setControl(
        motionMagic
            .withPosition(degreesToMotorRotations(refrenceDegrees))
            .withSlot(0)
            .withFeedForward(calculateFeedForward()));
  }

  private double calculateFeedForward() {
    return Math.sin(Math.toRadians(getDegrees() - 0)) * (ARM_FEED_FWD);
  }

  private double getDegrees() {
    return motorRotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  public static double motorRotationsToDegrees(double rotations) {
    return (rotations / ARM_GEAR_RATIO) * 360;
  }

  public static double degreesToMotorRotations(double degrees) {
    return (degrees * ARM_GEAR_RATIO) / 360;
  }

  public double getReferenceDegrees() {
    return refrenceDegrees;
  }

  public double getError() {
    return getReferenceDegrees() - getDegrees();
  }

  public boolean isAtSetpoint(double toleranceDegrees) {
    return Math.abs(getError()) < toleranceDegrees; // used to be 1.0
  }

  public boolean isAtSetpoint() {
    return isAtSetpoint(1.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Degrees", this::getDegrees, null);
    builder.addDoubleProperty("Reference", this::getReferenceDegrees, null);
    builder.addDoubleProperty("Error", this::getError, null);
    builder.addDoubleProperty("Feed Forward", this::calculateFeedForward, null);
    builder.addBooleanProperty("Is At Setpoint", this::isAtSetpoint, null);
  }
}
