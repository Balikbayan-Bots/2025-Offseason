package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.BodySetpoint;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.ClawState;
import frc.robot.subsystems.manipulators.ClawSubsystem;
import frc.robot.subsystems.manipulators.IntakeSetpoint;
import frc.robot.subsystems.manipulators.IntakeState;
import frc.robot.subsystems.manipulators.IntakeSubsytem;

public class ManipulatorCommands {

  private static ClawSubsystem claw = ClawSubsystem.getInstance();
  private static IntakeSubsytem intake = IntakeSubsytem.getInstance();
  private static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private static ArmSubsystem arm = ArmSubsystem.getInstance();

  private static Command setClawState(ClawState state) {
        return new RunCommand(
            () -> {
              claw.setState(state);
            },
            claw);
  }

  private static Command setIntakeState(IntakeState state) {
    return new InstantCommand(
        () -> {
          intake.setState(state);
        },
        intake);
  }

  public static Command intakeSetpointRun(IntakeSetpoint setpoint) {
    return new RunCommand(
        () -> {
          intake.updateSetpoint(setpoint);
        },
        intake);
  }

  public static Command stopIntake() {
    return setClawState(ClawState.IDLE);
  }

  public static Command holdAlgae() {
    return setClawState(ClawState.HOLDING_ALGAE);
  }

  public static Command runIntake() {
    return new RunCommand(() -> claw.setState(ClawState.INTAKE), claw);
  }

  public static Command runOutake() {
    return setClawState(ClawState.OUTAKE);
  }

  public static Command beamIntake() {
    return new RunCommand(() -> claw.setState(ClawState.INTAKE), claw)
        .until(() -> claw.getBeamBreak());
  }

  public static Command score() {
    return new SequentialCommandGroup(
        new SelectCommand<>(
            Map.of(
            BodySetpoint.CORAL_LEVEL2, BodyCommands.armSetpointRun(BodySetpoint.SCORE_LEVEL2),
            BodySetpoint.CORAL_LEVEL3, BodyCommands.armSetpointRun(BodySetpoint.SCORE),
            BodySetpoint.CORAL_LEVEL4, BodyCommands.armSetpointRun(BodySetpoint.SCORE)
            ),
            () -> arm.getCurrentSetpoint()),
        new WaitCommand(0.1),
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                setClawState(ClawState.SCORE).withTimeout(0.001),
                SwerveCommands.backUp().withTimeout(1.0)
            ),
            new ParallelCommandGroup(
            BodyCommands.elevSetpointRun(BodySetpoint.SAFE_STOW).until(elevator::isAtSetpoint),
            BodyCommands.armSetpointRun(BodySetpoint.SAFE_STOW).until(arm::isAtSetpoint)
            )
        ),
        new ParallelCommandGroup(
            stopIntake().withTimeout(0.001),
            BodyCommands.armSetpointRun(BodySetpoint.STOW_POS).until(arm::isAtSetpoint),
            BodyCommands.elevSetpointRun(BodySetpoint.STOW_POS).until(elevator::isAtSetpoint)
        ));
  }

  public static Command scoreLevelOne() {
    return new SequentialCommandGroup(
        new RunCommand(() -> intake.setState(IntakeState.SHOOT)).withTimeout(0.5),
        intakeSetpointRun(IntakeSetpoint.STOWED_HANDOFF).until(intake::isAtSetpoint),
        setIntakeState(IntakeState.START));
  }

  public static Command intakeLevelOne() {

    return intakeSetpointRun(IntakeSetpoint.LVL_ONE);
  }

  public static Command intakeLevelHandoff() {
    return new SequentialCommandGroup(
        intakeSetpointRun(IntakeSetpoint.STOWED_HANDOFF).until(intake::isAtSetpoint),
        setIntakeState(IntakeState.HOLD));
  }

  public static Command groundIntake() {
    return new SequentialCommandGroup(
        intakeSetpointRun(IntakeSetpoint.DEPLOYED).withTimeout(0.25),
        new RunCommand(() -> intake.setState(IntakeState.INTAKE), intake));
  }

  public static Command handoff() {
    return new ParallelCommandGroup(
        setIntakeState(IntakeState.HANDOFF), BodyCommands.positionHandoff());
  }

  public static Command handover() {
    return new SequentialCommandGroup(
        BodyCommands.positionHandoff(),

        // new WaitCommand(4.0), // TODO: BANDIAD FIX FOR ISATSETPOINT

        new ParallelDeadlineGroup(beamIntake(), setIntakeState(IntakeState.HANDOFF)),
        stopIntake().withTimeout(0.5),
        setIntakeState(IntakeState.START));
  }
}
