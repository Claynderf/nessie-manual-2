package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.*;

//import frc.robot.subsystems.PWMDrivetrain;



public final class Autos {
  private static DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

  /** Example static factory for an autonomous command. */
  public static Command AutoCommand() {
    return new RunCommand(() -> m_drivetrain.driveCartesian(-0.5, 0 , 0), m_drivetrain)
          .withTimeout(1)
          .andThen(new RunCommand(() -> m_drivetrain.driveCartesian(0, 0, 0), m_drivetrain ));
  }
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
