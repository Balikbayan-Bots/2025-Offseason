package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BodyCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.body.BodySetpoint;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Telemetry;

public class Bindings {
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private Bindings() {
    throw new IllegalStateException("Utility class");
  }

  public static void configureSwerveBinds() {
    swerve.setDefaultCommand(SwerveCommands.manualDrive(0.1));

    Controls.Swerve.reorient.onTrue(SwerveCommands.reorient());

    // Controls.Swerve.test
    // .onTrue(SwerveCommands.driveToPose(new Pose2d(16.25, 6.85,
    // Rotation2d.fromDegrees(142.286))));

    Telemetry logger = new Telemetry(SwerveConstants.SPEED_AT_12V.in(MetersPerSecond));
    swerve.registerTelemetry(logger::telemeterize);
  }

  public static void configureClawBinds() {
    Controls.Manipulators.intake.whileTrue(ManipulatorCommands.beamIntake());
    // .onFalse(ManipulatorCommands.stopIntake());
    // Controls.Manipulators.outake
    //     .whileTrue(ManipulatorCommands.runOutake())
    //     .onFalse(ManipulatorCommands.stopIntake());

    // Controls.Manipulators.score.onTrue(score()).onFalse(ManipulatorCommands.stopIntake());
  }

  public static void configureBodyBinds() {
    Controls.Setpoint.stow.onTrue(BodyCommands.positionStow());
    Controls.Setpoint.lvlOne.onTrue(ManipulatorCommands.intakeLevelOne());
    Controls.Setpoint.lvlTwo.onTrue(BodyCommands.positionLevelTwo());
    Controls.Setpoint.lvlThree.onTrue(BodyCommands.positionLevelThree());
    Controls.Setpoint.lvlFour.onTrue(BodyCommands.positionLevelFour());
  }

  public static void configureIntakeBinds() {
    Controls.Manipulators.intakeLevelOne.onTrue(intakeLevelOne());
    Controls.Manipulators.groundIntake
        .whileTrue(groundIntake())
        .onFalse(ManipulatorCommands.intakeLevelHandoff());
    Controls.Manipulators.handOverIntake.whileTrue(ManipulatorCommands.handover());
  }

  public static Command score() {
    return new SequentialCommandGroup(
        BodyCommands.armSetpointRun(BodySetpoint.SCORE),
        ManipulatorCommands.score(),
        new WaitCommand(0.5),
        BodyCommands.positionStow());
  }

  public static Command intakeLevelOne() {
    return ManipulatorCommands.intakeLevelOne();
  }

  public static Command groundIntake() {
    return ManipulatorCommands.groundIntake();
  }
}
