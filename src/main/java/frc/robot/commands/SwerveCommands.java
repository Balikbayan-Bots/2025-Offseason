package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.controls.Controls;
import frc.robot.subsystems.body.BodyConstants;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.IntakeSetpoint;
import frc.robot.subsystems.manipulators.IntakeState;
import frc.robot.subsystems.manipulators.IntakeSubsytem;
import frc.robot.subsystems.swerve.SwerveConstants;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_ROT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_SPEED;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.LimelightSubsystem;

public class SwerveCommands {
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private static IntakeSubsytem intake = IntakeSubsytem.getInstance();

  private static SwerveRequest.RobotCentric limlightDrive = new SwerveRequest.RobotCentric()
    .withDeadband(SwerveConstants.MAX_TELEOP_SPEED*.01)
    .withRotationalDeadband(SwerveConstants.MAX_TELEOP_ROT*.01)
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
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    return swerve.applyRequest(
        () ->
            drive
                .withVelocityX(-0.25)
                .withVelocityY(0.5)
                .withRotationalRate(0));
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

  public static Command limelightIntakeCommand(LimelightSubsystem subsys, int pipeline){
    return new SequentialCommandGroup(
        new InstantCommand(()-> subsys.setPipeline(pipeline)),
        new ParallelCommandGroup(
        swerve.applyRequest(()-> limlightDrive
        .withVelocityY(-subsys.alignTy() * getDriveMultiplier(true))
        .withVelocityX(-subsys.alignTx() * getDriveMultiplier(true))
       .withRotationalRate(-subsys.alignRz())),
        intakeSetpointRun(IntakeSetpoint.DEPLOYED)
        )
    );
  }

    public static double getDriveMultiplier(boolean limelight){
        double currentPos = elevator.getInches();
        double maxPos = BodyConstants.ELEV_MAX_POS;
        double ratio = Math.abs(currentPos/maxPos);
        double speedMultiplier;

        if(limelight){
            speedMultiplier = 100D-50D*ratio;
        }else{

            speedMultiplier = 100D-70D*ratio;
        }

        return speedMultiplier/100D;
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
