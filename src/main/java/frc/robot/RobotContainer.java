// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NeoMotorDriveSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /// SUBSYSTEMS ///
  // Remember these are members of the class meaning they should start with the m_ prefix and end
  // with the Subsystem suffix
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final NeoMotorDriveSystem m_NeoMotorDriveSystem = new NeoMotorDriveSystem();
  private final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();

  /// CONTROLLERS ///
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    /// DEFAULT COMMANDS ///
    /*
     *  m_subsystemExample.setDefaultCommand(
     *    Commands.run(() -> m_subsystemExample.exampleCommand(args*), m_subsystemExample));
     */

    m_NeoMotorDriveSystem.setDefaultCommand(
        Commands.run(
            () ->
                m_NeoMotorDriveSystem.driveArcade(
                    m_driverController.getLeftY(), m_driverController.getRightX()),
            m_NeoMotorDriveSystem));

    // m_tankDriveSubsystem.setDefaultCommand(
    //     Commands.run(
    //         () ->
    //             m_tankDriveSubsystem.driveArcade(
    //                 m_driverController.getLeftY(), m_driverController.getLeftX()),
    //         m_tankDriveSubsystem));

    // m_tankDriveSubsystem.setDefaultCommand(
    //     Commands.run(
    //         () ->
    //             m_tankDriveSubsystem.driveCurvature(
    //                 m_driverController.getLeftY(), m_driverController.getRightX()),
    //         m_tankDriveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /// BOUND COMMANDS ///
    /*
     *     Continuous Command
     *     m_driverController.y().onTrue(new ExampleCommand(args*));
     *
     *     Single Input Command / Do not use a Command class
     *     m_driverController.y().onTrue(Commands.runOnce(() -> m_subsystemExample.exampleMethod(), m_subsystemExample));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}