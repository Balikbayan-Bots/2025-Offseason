package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.BodyCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwervePositions;
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

    // Controls.Debug.megatagTest.onTrue(SwerveCommands.driveTagNineLeft());
    Controls.Swerve.slow.onTrue(
      new InstantCommand(() -> {swerve.toggleSlowMode();}, swerve)
    );
    Controls.Swerve.rightPeg.onTrue(SwerveCommands.driveToPegProxy(SwervePositions.alignMent.RIGHT));
    Controls.Swerve.leftPeg.onTrue(SwerveCommands.driveToPegProxy(SwervePositions.alignMent.LEFT));
    Controls.Swerve.center.onTrue(SwerveCommands.driveToPegProxy(SwervePositions.alignMent.CENTER));

    // Controls.Swerve.test
    // .onTrue(SwerveCommands.driveToPose(new Pose2d(16.25, 6.85,
    // Rotation2d.fromDegrees(142.286))));

    Telemetry logger = new Telemetry(SwerveConstants.SPEED_AT_12V.in(MetersPerSecond));
    swerve.registerTelemetry(logger::telemeterize);
    // Controls.Swerve.rightPeg.onTrue((SwerveCommands.rightPegLineup()));
  }

  public static void configureClawBinds() {
    Controls.Manipulators.intake.whileTrue(ManipulatorCommands.beamIntake());
    Controls.Manipulators.intakeAlgae.onTrue(ManipulatorCommands.beamAlgaeIntake());
    Controls.Manipulators.scoreAlgae.onTrue(ManipulatorCommands.algaeScore());
    // .onFalse(ManipulatorCommands.stopIntake());
    // Controls.Manipulators.outake
    //     .whileTrue(ManipulatorCommands.runOutake())
    //     .onFalse(ManipulatorCommands.stopIntake());

    Controls.Manipulators.score.onTrue(ManipulatorCommands.score());
    // .onFalse(ManipulatorCommands.stopIntake());
  }

  public static void configureBodyBinds() {
    Controls.Setpoint.stowUp.onTrue(BodyCommands.positionStow());
    Controls.Setpoint.stowLow.onTrue(BodyCommands.positionStart());
    Controls.Setpoint.lvlTwo.onTrue(BodyCommands.positionLevelTwo());
    Controls.Setpoint.lvlThree.onTrue(BodyCommands.positionLevelThree());
    Controls.Setpoint.lvlFour.onTrue(BodyCommands.positionLevelFour());
    Controls.Setpoint.algaeHigh.onTrue(BodyCommands.positionHighAlgae());
    Controls.Setpoint.algaeLow.onTrue(BodyCommands.positionLowAlgae());
    Controls.Setpoint.netPos.onTrue(BodyCommands.positionNet());
    Controls.Setpoint.algaeFloor.onTrue(BodyCommands.positionFloorAlgae());
  }

  public static void configureIntakeBinds() {
    Controls.Manipulators.intakeLevelOne.onTrue(intakeLevelOne());
    Controls.Manipulators.groundIntake
        .whileTrue(groundIntake())
        .onFalse(ManipulatorCommands.intakeLevelHandoff());
    Controls.Manipulators.handOverIntake.onTrue(ManipulatorCommands.handover());
    Controls.Manipulators.scoreLevelOne.onTrue(ManipulatorCommands.scoreLevelOne());
  }

  // public static Command score() {
  // return new SequentialCommandGroup(
  //   BodyCommands.armSetpointRun(BodySetpoint.SCORE),
  // ManipulatorCommands.score(),
  //       n/ew WaitCommand(0.5),
  //       BodyCommands.positionStow());
  // }

  public static Command intakeLevelOne() {
    return ManipulatorCommands.intakeLevelOne();
  }

  public static Command groundIntake() {
    return ManipulatorCommands.groundIntake();
  }
}
