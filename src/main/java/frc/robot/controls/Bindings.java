package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BodyCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.vision.IntakeLimelightSubsystem;

public class Bindings {
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private static IntakeLimelightSubsystem IntakeLimelight = IntakeLimelightSubsystem.getInstance();

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

    Controls.Swerve.LimelightIntake.whileTrue(
            SwerveCommands.aimAndDriveToTarget(IntakeLimelight))
        .onFalse(ManipulatorCommands.intakeLevelHandoff());
  }

  public static void configureClawBinds() {
    Controls.Manipulators.intake.whileTrue(ManipulatorCommands.beamIntake());
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
