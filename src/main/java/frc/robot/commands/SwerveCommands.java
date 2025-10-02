package frc.robot.commands;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_ROT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_SPEED;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.Controls;
import frc.robot.subsystems.body.BodyConstants;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.IntakeSetpoint;
import frc.robot.subsystems.manipulators.IntakeState;
import frc.robot.subsystems.manipulators.IntakeSubsytem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.IntakeLimelightSubsystem;
import frc.robot.vision.IntakeLimelightSubsystem.AimingState;
import frc.robot.vision.LimelightConfigs;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.LimelightSubsystem;

public class SwerveCommands {
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private static IntakeSubsytem intake = IntakeSubsytem.getInstance();
  private static IntakeLimelightSubsystem intakeLimelight = IntakeLimelightSubsystem.getInstance();

  private double lastForwardCommand = 0.0;
  private double lastRotationCommand = 0.0;

  private double coastingTyError = 0.0;
  private double tyErrorTraveled = 0.0;
  private long lastTimeMillis = System.currentTimeMillis();

  private static SwerveRequest.RobotCentric limlightDrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(SwerveConstants.MAX_TELEOP_SPEED * .01)
          .withRotationalDeadband(SwerveConstants.MAX_TELEOP_ROT * .01)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private SwerveCommands() {
    throw new IllegalStateException("Utility class");
  }

  public static Command manualDrive(double deadband) {
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MAX_TELEOP_SPEED * deadband)
            .withRotationalDeadband(MAX_TELEOP_ROT * deadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    return swerve.applyRequest(
        () ->
            drive
                .withVelocityX(Controls.Swerve.translateX.get())
                .withVelocityY(Controls.Swerve.translateY.get())
                .withRotationalRate(Controls.Swerve.rotate.get()));
  }

  public static Command backUp() {
    SwerveRequest.RobotCentric drive =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    return swerve.applyRequest(
        () -> drive.withVelocityX(-0.25).withVelocityY(0.5).withRotationalRate(0));
  }

  public static Command driveToPose(Pose2d targetPosition) {
    PathConstraints constraints =
        new PathConstraints(1.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return driveToPose(targetPosition, constraints);
  }

  public static Command driveToPose(Pose2d targetPosition, PathConstraints constraints) {
    return AutoBuilder.pathfindToPose(targetPosition, constraints, 0.0);
  }

  public static Command reorient() {
    return swerve.runOnce(() -> swerve.seedFieldCentric());
  }

  public static Command limelightIntakeCommand(LimelightSubsystem subsys, int pipeline) {

    return new SequentialCommandGroup(
        new InstantCommand(() -> subsys.setPipeline(pipeline)),
        new ParallelCommandGroup(
            swerve.applyRequest(
                () ->
                    limlightDrive
                        // .withVelocityY(-subsys.alignTx() * getDriveMultiplier(true)*.7)
                        .withVelocityX(-subsys.alignTy() * getDriveMultiplier(true))
                        .withRotationalRate(-subsys.alignRz()))
            // intakeSetpointRun(IntakeSetpoint.DEPLOYED)
            ));
  }

  public static Command driveToVisionTarget() {
    PathConstraints constraints =
        new PathConstraints(
            SwerveConstants.SPEED_AT_12V.in(edu.wpi.first.units.Units.MetersPerSecond) * 0.75,
            4.0,
            Units.degreesToRadians(540),
            Units.degreesToRadians(720));

    // Desired offset from the center of the intake target to the robot's center
    // This makes the robot stop X meters away from the target
    final double TARGET_X_OFFSET = 0.5; // Example: Stop 0.5 meters away

    // This is the core logic: a command that is created *only* if a target is visible.
    Command targetDriveCommand = Commands.defer(
        () -> {
          // 1. Get the latest pose estimate from the Intake Limelight
          LimelightHelpers.PoseEstimate estimate =
              LimelightHelpers.getBotPoseEstimate_wpiBlue(
                  LimelightConfigs.IntakeLimelight.name());

          // 2. Check if the target is valid (using the recommended criteria)
          if (estimate.tagCount > 0 && intakeLimelight.getTv()) {
            // Target is valid, calculate the field target pose
            Pose2d currentTargetPose = estimate.pose;

            // Calculate the final target pose: current target X/Y, but offset by X_OFFSET along the approach vector
            Pose2d finalTargetPose = new Pose2d(
                currentTargetPose.getX() - TARGET_X_OFFSET * currentTargetPose.getRotation().getCos(),
                currentTargetPose.getY() - TARGET_X_OFFSET * currentTargetPose.getRotation().getSin(),
                currentTargetPose.getRotation()
            );

            // 3. Return the PathPlanner command to drive to the calculated field position
            return AutoBuilder.pathfindToPose(
                finalTargetPose,
                constraints,
                0.0 // Goal End Velocity (stop at the target)
            ).withTimeout(4.0); // Safety timeout

          } else {
            // Target not found or invalid: stop the robot immediately.
            System.out.println("Limelight target lost or invalid. Stopping movement.");
            // ApplyChassisSpeeds uses zero velocity if instantiated without arguments
            // We use .applyRequest to wrap the SwerveRequest in a runnable command.
            
            return swerve.applyRequest(() -> new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));    
    
          }
        },
        java.util.Set.of(swerve, intakeLimelight) // Requirements for the deferred command
    );

    // This command runs the target drive command (either pathfind or stop)
    // The command immediately checks the state upon starting (button press)
    return new ParallelCommandGroup(
        targetDriveCommand,
        intakeSetpointRun(IntakeSetpoint.DEPLOYED) // Run intake deploy parallel to driving
        );
  }


  /** Helper method to retrieve the latest valid MegaTag pose */
  private static Optional<PoseEstimate> getLatestValidTargetPose(String limelightName) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    
    // Check if a target is visible and the pose estimate is considered valid 
    // (using Megatag, we generally look for at least one tag)
    if (poseEstimate == null || poseEstimate.tagCount == 0 || poseEstimate.pose.equals(new Pose2d())) {
      return Optional.empty();
    }
    
    // Note: Ambiguity check (0.7) should ideally be done in the MegaTag subsystem periodic loop
    // before adding the measurement to odometry, but we can check here too if needed.
    
    return Optional.of(poseEstimate);
  }

 
  public static double getDriveMultiplier(boolean limelight) {
    double currentPos = elevator.getInches();
    double maxPos = BodyConstants.ELEV_MAX_POS;
    double ratio = Math.abs(currentPos / maxPos);
    double speedMultiplier;

    if (limelight) {
      speedMultiplier = 100D - 50D * ratio;
    } else {

      speedMultiplier = 100D - 70D * ratio;
    }

    return speedMultiplier / 100D;
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
}
