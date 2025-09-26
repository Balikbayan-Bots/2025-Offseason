package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.LimelightConfigs;
import frc.robot.vision.Megatag;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();

    Megatag.addLimelight(LimelightConfigs.ReefLimelight);
    Megatag.addLimelight(LimelightConfigs.IntakeLimelight);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Megatag.updateAllOdometry();
  }

  @Override
  public void autonomousInit() {
    autoCommand = robotContainer.getAutonomousCommand();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
