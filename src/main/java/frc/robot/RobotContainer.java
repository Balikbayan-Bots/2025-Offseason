package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Bindings;
import frc.robot.controls.OperatorInterface;
import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.ClawSubsystem;
import frc.robot.subsystems.manipulators.IntakeSubsytem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  // Declare Controls
  public final OperatorInterface oi;

  // Declare Subsystems
  public final SwerveSubsystem swerve;
  public final ClawSubsystem claw;
  public final ArmSubsystem arm;
  public final ElevatorSubsystem elevator;
  public final IntakeSubsytem intake;
  // Declare Choosers
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize Controls
    oi = OperatorInterface.getInstance();

    // Initialize Subsystems
    swerve = SwerveSubsystem.getInstance();
    arm = ArmSubsystem.getInstance();
    elevator = ElevatorSubsystem.getInstance();
    claw = ClawSubsystem.getInstance();
    intake = IntakeSubsytem.getInstance();
    // Initialize Choosers
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    configureDashboard();
  }

  private void configureBindings() {
    Bindings.configureSwerveBinds();
    Bindings.configureClawBinds();
    Bindings.configureBodyBinds();
    Bindings.configureIntakeBinds();
  }

  private void configureDashboard() {
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(claw);
    SmartDashboard.putData(intake);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
