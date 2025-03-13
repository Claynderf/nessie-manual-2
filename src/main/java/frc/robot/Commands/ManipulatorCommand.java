package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class ManipulatorCommand extends Command{
    private ManipulatorSubsystem m_manipulatorSubsystem;
    private boolean isIntake = false;
    private CommandXboxController m_driverController;

    public ManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, boolean isIntake, CommandXboxController driverController)
    {
        m_manipulatorSubsystem = manipulatorSubsystem;
        isIntake = isIntake;
        this.m_driverController = driverController;

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void execute()
    {
       
    }

    @Override
    public void end(boolean interrupted)
    {
      
    }
}
