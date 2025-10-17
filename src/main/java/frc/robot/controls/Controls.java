package frc.robot.controls;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_ROT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_SPEED;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

public class Controls {
  private static OperatorInterface oi = OperatorInterface.getInstance();
  private static CommandXboxController driver = oi.getDriver();
  private static CommandXboxController coDriver = oi.getCoDriver();

  private Controls() {
    throw new IllegalStateException("Utility class");
  }

  public final class Swerve {
    private Swerve() {
      throw new IllegalStateException("Utility class");
    }

    /*** Driver 'Left X' */
    public static final Supplier<Double> translateX =
        () -> {
          return -driver.getLeftY() * MAX_TELEOP_SPEED;
        };

    /*** Driver 'Left Y' */
    public static final Supplier<Double> translateY =
        () -> {
          return -driver.getLeftX() * MAX_TELEOP_SPEED;
        };

    /*** Driver 'Right X' */
    public static final Supplier<Double> rotate =
        () -> {
          return -driver.getRightX() * MAX_TELEOP_ROT;
        };

    /*** Driver 'Start' */
    public static final Trigger reorient = driver.start();
    // public static final Trigger rightPeg = driver.b();
  }

  public final class Debug {
    private Debug() {
      throw new IllegalStateException("Utility class");
    }

    public static final Trigger magnetRezero = driver.povUpRight();
    public static final Trigger nudgeDown = coDriver.povRight();
    public static final Trigger nudgeUp = coDriver.povLeft();
    public static final Trigger megatagTest = driver.a();
  }

  public class Setpoint {
    private Setpoint() {
      throw new IllegalStateException("Utility class");
    }

    // public static final Trigger lvlOne = coDriver.povUp();

    public static final Trigger lvlTwo = coDriver.a();

    public static final Trigger lvlThree = coDriver.b();

    public static final Trigger lvlFour = coDriver.y();

    // set to upper stow for scoring L4-L2 and Algae
    public static final Trigger stowUp = coDriver.povUp();
    // set to lower stow for scoring L1 with Intake
    public static final Trigger stowLow = coDriver.povDown();
    public static final Trigger algaeHigh = coDriver.povRight();
    public static final Trigger netPos = coDriver.povLeft();
  }

  public class Body {
    private Body() {
      throw new IllegalStateException("Utility class");
    }

    public static final Trigger elevRezero = driver.povDown();
  }

  public class Climb {
    private Climb() {
      throw new IllegalStateException("Utility class");
    }

    public static final Trigger deploy = coDriver.back();
  }

  public class Manipulators {
    private Manipulators() {
      throw new IllegalStateException("Utility class");
    }

    public static final Trigger intake = coDriver.rightBumper();
    public static final Trigger score = driver.rightTrigger();
    public static final Trigger scoreLevelOne = driver.rightBumper();
    public static final Trigger handOverIntake = coDriver.leftBumper();
    public static final Trigger scoreAlgae = driver.y();

    public static final Trigger intakeLevelOne = coDriver.x();
    public static final Trigger groundIntake = driver.leftBumper();
    public static final Trigger intakeAlgae = driver.leftTrigger();
  }
}
