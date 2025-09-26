package frc.robot.subsystems.body;

import static frc.robot.subsystems.body.BodyConstants.ELEV_FEED_FWD;
import static frc.robot.subsystems.body.BodyConstants.ELEV_GEAR_RATIO;
import static frc.robot.subsystems.body.BodyConstants.ELEV_MAX_VOLATGE_REVERSE;
import static frc.robot.subsystems.body.BodyConstants.ELEV_MAX_VOLTAGE_FWD;
import static frc.robot.subsystems.body.BodyConstants.ELEV_MOTION_MAGIC_CONFIGS;
import static frc.robot.subsystems.body.BodyConstants.ELEV_MOTOR_LEFT;
import static frc.robot.subsystems.body.BodyConstants.ELEV_MOTOR_RIGHT;
import static frc.robot.subsystems.body.BodyConstants.ELEV_SLOT_ZERO;
import static frc.robot.subsystems.body.BodyConstants.ELEV_SPROCKET_CIRCUMFERENCE;
import static frc.robot.subsystems.body.BodyConstants.kElevLimits;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem m_instance;

  public static ElevatorSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ElevatorSubsystem();
    }
    return m_instance;
  }

  private BodySetpoint activeSetpoint = BodySetpoint.STOW_INTAKE;
  private double referenceInches = 0;
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private MotionMagicVoltage motionMagic;

  public ElevatorSubsystem() {
    leftMotor = new TalonFX(ELEV_MOTOR_LEFT);
    rightMotor = new TalonFX(ELEV_MOTOR_RIGHT);
    configureElev(leftMotor, null);
    configureElev(rightMotor, leftMotor);
    reZero();
    motionMagic = new MotionMagicVoltage(0).withSlot(0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        200, leftMotor.getPosition(), rightMotor.getPosition());
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftMotor.getSupplyVoltage(),
        leftMotor.getFault_Hardware(),
        leftMotor.getMotorVoltage(),
        leftMotor.getSupplyCurrent(),
        leftMotor.getStatorCurrent(),
        leftMotor.getFault_DeviceTemp(),
        rightMotor.getSupplyVoltage(),
        rightMotor.getFault_Hardware(),
        rightMotor.getMotorVoltage(),
        rightMotor.getSupplyCurrent(),
        rightMotor.getStatorCurrent(),
        rightMotor.getFault_DeviceTemp());

    leftMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();
  }

  private void configureElev(TalonFX motor, TalonFX leaderMotor) {
    TalonFXConfiguration newConfig = new TalonFXConfiguration();
    if (leaderMotor != null) {
      motor.setControl(new Follower(leaderMotor.getDeviceID(), true));
    }
    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = kElevLimits.forwardLimit();

    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kElevLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kElevLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = ELEV_MAX_VOLTAGE_FWD; // Down
    voltage.PeakReverseVoltage = ELEV_MAX_VOLATGE_REVERSE; // Up

    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = ELEV_SLOT_ZERO[0];
    slot0.kI = ELEV_SLOT_ZERO[1];
    slot0.kD = ELEV_SLOT_ZERO[2];
    slot0.kS = ELEV_SLOT_ZERO[3];
    slot0.kG = ELEV_SLOT_ZERO[4];
    slot0.kV = ELEV_SLOT_ZERO[5];
    slot0.kA = ELEV_SLOT_ZERO[6];

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    var output = newConfig.MotorOutput;
    output.NeutralMode = NeutralModeValue.Brake;
    motionMagic.MotionMagicAcceleration = ELEV_MOTION_MAGIC_CONFIGS[0];
    motionMagic.MotionMagicCruiseVelocity = ELEV_MOTION_MAGIC_CONFIGS[1];
    motionMagic.MotionMagicJerk = ELEV_MOTION_MAGIC_CONFIGS[2];
    motor.getConfigurator().apply(newConfig);
  }

  public void updateSetpoint(BodySetpoint setPoint) {
    activeSetpoint = setPoint;
    updateReference(activeSetpoint.getElevTravel());
  }

  public void updateReference(double inches) {
    referenceInches = inches;
  }

  public void reZero() {
    leftMotor.setPosition(0);
  }

  public double getRotations() {
    return (leftMotor.getPosition().getValueAsDouble());
  }

  public double getReferenceInches() {
    return referenceInches;
  }

  public double getError() {
    return getReferenceInches() - getInches();
  }

  public double getInches() {
    return motorRotationsToInches(getRotations());
  }

  public static double inchesToMotorRotations(double inches) {
    return (inches / ELEV_SPROCKET_CIRCUMFERENCE) * ELEV_GEAR_RATIO;
  }

  public static double motorRotationsToInches(double rotations) {
    return (rotations / ELEV_GEAR_RATIO) * ELEV_SPROCKET_CIRCUMFERENCE;
  }

  public boolean isAtSetpoint(double toleranceInches) {
    return Math.abs(getError()) < toleranceInches; // used to be 2.5
  }

  public boolean isAtSetpoint() {
    return isAtSetpoint(2.5);
  }

  @Override
  public void periodic() {
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    updateReference(activeSetpoint.getElevTravel());
    leftMotor.setControl(
        motionMagic
            .withPosition(inchesToMotorRotations(referenceInches))
            .withSlot(0)
            .withFeedForward(ELEV_FEED_FWD));
    // leftMotor.setControl(new CoastOut());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");
    builder.addDoubleProperty("Rotations", this::getRotations, null);
    builder.addDoubleProperty("Refrence", this::getReferenceInches, null);
    builder.addDoubleProperty("Error", this::getError, null);
    builder.addDoubleProperty("Inches", this::getInches, null);
    builder.addBooleanProperty("Is At Setpoint", this::isAtSetpoint, null);
  }
}
