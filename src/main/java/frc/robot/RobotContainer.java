package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.*;
//import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
//import frc.robot.Commands.ManipulatorCommand;
import frc.robot.Commands.StopCommand;
import frc.robot.Commands.Autos;
//import frc.robot.Commands.HoldCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.
  ;

  // The robot's subsystems are defined here.
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
  private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /*
   * The gamepad provided in the KOP shows up like an XBox controller if the mode
   * switch is set to X mode using the
   * switch on the top.
   */
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    // configureBindings();
   
      
      autoChooser.setDefaultOption("Default Auto", new RunCommand(() -> {}));
      autoChooser.addOption("forward and shoot", Autos.AutoCommand(m_drivetrain));
    autoChooser.addOption("hi bob" , new PrintCommand( "hi bob"));
    SmartDashboard.putData("Auto mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * accessed via the
   * named factory methods in the Command* classes in
   * edu.wpi.first.wpilibj2.command.button (shown
   * below) or via the Trigger constructor for arbitary conditions
   */
  /*
   * private void configureBindings() {
   * 
   * m_drivetrain.setDefaultCommand(
   * new RunCommand(
   * () ->
   * m_drivetrain.driveCartesian(
   * -m_driverController.getLeftY() , -m_driverController.getLeftX()
   * ,-m_driverController.getRightX() ),
   * m_drivetrain));
   * 
   * 
   * 
   * }
   */
  public void drive() {
    var x = ConditionJoystickValue(-m_driverController.getLeftY());
    var y = ConditionJoystickValue(m_driverController.getLeftX());
    var rot = ConditionJoystickValue(m_driverController.getRightX());
    m_drivetrain.driveCartesian(x, y, rot);
  }

  private double ConditionJoystickValue(double axisValue) {
    if (Math.abs(axisValue) < 0.05) {
      return 0;
    }

    var speedReduction = 1;

    return axisValue * axisValue * axisValue * speedReduction;
  }

 /*  public void testdrive() {

    m_drivetrain.driveCartesian(-0.5, 0, 0);
  }*/

  public void directDriveElevator() {
    if (m_driverController.povDown().getAsBoolean()) {
      m_elevator.moveElevator(ConditionJoystickValue(-m_driverController.getRightY()));
    } else {
      m_elevator.moveElevator(0);
    }
    }
  // if (m_driverController.getAButton())
  // {
  // m_elevator.goToPosition(0);
  // }
  // else if (m_driverController.getXButton())
  // {
  // m_elevator.goToPosition(5);
  // }

  public void directWristDrive() {
 
    if (m_driverController.getLeftTriggerAxis() > 0.25)  {
      if (m_wrist.m_wristMotor.getEncoder().getPosition() < 25){
       m_wrist.moveWrist(m_driverController.getLeftTriggerAxis() / 5);
      }
        
    }
    else if (m_driverController.getRightTriggerAxis() > 0.25){
      if (m_wrist.m_wristMotor.getEncoder().getPosition() > 0){
        m_wrist.moveWrist(-m_driverController.getRightTriggerAxis() / 5);
      }
      
    }
    else {
      m_wrist.moveWrist(0);
    }
   
 
  }

  public void registerTriggers() {
    m_driverController.leftBumper().whileTrue(new RunCommand(() -> m_manipulator.intake(0.99), m_manipulator));

  }

  public void DirectManipulatorDrive() {
    // m_manipulator.intake(m_driverController.rightBumper().getAsBoolean()?
    // 0.75:0);
    // m_manipulator.shoot(m_driverController.leftBumper().getAsBoolean()? -0.75:0);

    if (m_driverController.rightBumper().getAsBoolean()) {
      m_manipulator.intake(0.50);
    } else if (m_driverController.leftBumper().getAsBoolean()) {
      m_manipulator.intake(-0.50);

    } else {
      m_manipulator.intake(0);
    }

  }

  public void dashboardStuff() {
    m_elevator.dashboardStuff();
    m_wrist.dashboardStuff();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}