package frc.robot.subsystems.manipulators;

import static frc.robot.subsystems.manipulators.ManipulatorConstants.BEAM_BREAK_ID;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.CLAW_MAX_VOLTAGE_FORWARD;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.CLAW_MAX_VOLTAGE_REVERSE;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.CLAW_MOTOR_ID;
import static frc.robot.subsystems.manipulators.ManipulatorConstants.kclawLimits;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  public static ClawSubsystem m_instance;

  public static ClawSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ClawSubsystem();
    }
    return m_instance;
  }

  private ClawState state = ClawState.IDLE;

  private TalonFX motor;
  private DigitalInput beamBreak;

  private ClawSubsystem() {
    motor = new TalonFX(CLAW_MOTOR_ID);
    beamBreak = new DigitalInput(BEAM_BREAK_ID);
    configureclaw(motor);
    BaseStatusSignal.setUpdateFrequencyForAll(200, motor.getPosition(), motor.getVelocity());
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

  private void configureclaw(TalonFX motor) {
    TalonFXConfiguration newConfig = new TalonFXConfiguration();
    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = false;
    limits.ReverseSoftLimitEnable = false;

    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kclawLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kclawLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = CLAW_MAX_VOLTAGE_FORWARD; // Down
    voltage.PeakReverseVoltage = CLAW_MAX_VOLTAGE_REVERSE; // Up
  }

  public boolean getBeamBreak() {
    return !beamBreak.get();
  }

  public double getSpeed() {
    return state.getSpeed();
  }

  public ClawState getState() {
    return state;
  }

  @Override
  public void periodic() {
    motor.set(state.getSpeed());
  }

  public void setState(ClawState state) {
    this.state = state;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Claw");
    builder.addBooleanProperty("Beam", this::getBeamBreak, null);
    builder.addDoubleProperty("Claw Speed", this::getSpeed, null);
  }
}
