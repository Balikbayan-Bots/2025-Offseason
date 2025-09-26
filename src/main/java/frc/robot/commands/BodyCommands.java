package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.BodySetpoint;
import frc.robot.subsystems.body.ElevatorSubsystem;

public class BodyCommands {

  private static ElevatorSubsystem elev = ElevatorSubsystem.getInstance();
  private static ArmSubsystem arm = ArmSubsystem.getInstance();

  public static Command armSetpointRun(BodySetpoint setpoint) {
    return new InstantCommand(
        () -> {
          arm.updateSetpoint(setpoint);
        },
        arm);
  }

  public static Command elevSetpointRun(BodySetpoint setpoint) {
    return new InstantCommand(
        () -> {
          elev.updateSetpoint(setpoint);
        },
        elev);
  }

  public static Command positionHandoff() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.HANDOFF), armSetpointRun(BodySetpoint.HANDOFF)
        // elevSetpointRun(BodySetpoint.HANDOFF).until(() -> elev.isAtSetpoint()), //TODO:
        // isAtSetpoint doesnt work
        // armSetpointRun(BodySetpoint.HANDOFF).until(() -> arm.isAtSetpoint())
        );
  }

  public static Command positionLevelTwo() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.CORAL_LEVEL2).until(() -> elev.isAtSetpoint(0.5)),
        armSetpointRun(BodySetpoint.CORAL_LEVEL2).until(() -> arm.isAtSetpoint(0.5)));
  }

  public static Command positionLevelThree() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.CORAL_LEVEL3).until(() -> elev.isAtSetpoint(0.5)),
        armSetpointRun(BodySetpoint.CORAL_LEVEL3).until(() -> arm.isAtSetpoint(0.5)));
  }

  public static Command positionLevelFour() {
    return new SequentialCommandGroup(
        // elevSetpointRun(BodySetpoint.CORAL_LEVEL4),
        // armSetpointRun(BodySetpoint.CORAL_LEVEL4)
        elevSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> elev.isAtSetpoint(0.5)),
        armSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> arm.isAtSetpoint(0.5)));
  }

  public static Command positionStow() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.STOW_INTAKE).until(elev::isAtSetpoint),
        armSetpointRun(BodySetpoint.STOW_INTAKE).until(arm::isAtSetpoint));
  }
}
