package frc.robot.vision;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.Megatag.LimelightConfig;
import java.util.ArrayList;
import java.util.List;

public class ObjectDetection {
  private static List<LimelightConfig> limelights = new ArrayList<>();
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
}
