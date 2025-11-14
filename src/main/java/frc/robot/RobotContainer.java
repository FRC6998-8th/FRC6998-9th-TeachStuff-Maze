// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.MazeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final MazeSubsystem m_MazeSubsystem = new MazeSubsystem("Roll");
  private final ExampleCommand m_ExampleCommand = new ExampleCommand(m_MazeSubsystem);

  // private final ExampleCommand exampleCommand = new ExampleCommand(m_MazeSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    m_MazeSubsystem.setDefaultCommand(m_ExampleCommand);
    m_driverController.b().whileTrue(m_MazeSubsystem.ResetDriveGyro());
    m_driverController.a().onTrue(m_MazeSubsystem.ResetMaze());
    
    m_driverController.povDown().whileTrue(m_MazeSubsystem.SysIdDynamic(Direction.kReverse));
    m_driverController.povUp().whileTrue(m_MazeSubsystem.SysIdDynamic(Direction.kForward));
    m_driverController.povLeft().whileTrue(m_MazeSubsystem.SysIdQuastatic(Direction.kReverse));
    m_driverController.povUp().whileTrue(m_MazeSubsystem.SysIdQuastatic(Direction.kForward));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
