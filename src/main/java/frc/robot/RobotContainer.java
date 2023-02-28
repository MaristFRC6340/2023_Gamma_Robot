// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RampClimbBangBang;
import frc.robot.commands.RampClimbSequence;
import frc.robot.commands.WristRotatePIDTestCommand;
import frc.robot.commands.WristRotateTestCommand;
import frc.robot.commands.WristTeleopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GammaDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final GammaDriveSubsystem m_robotDrive = new GammaDriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final DriveCommand driveCommand = new DriveCommand(m_robotDrive);
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();

  private final ArmCommand armCommand = new ArmCommand(m_armSubsystem);

  // The driver's controller
  // TODO: Move this to a Static field in Robot Class
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Below commented out for refactoring
    /* 
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftY()*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftX()*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getRightX()*DriveConstants.SpeedMultiplier, 0.06),
                true),
            m_robotDrive));
    */
  }

  private void configureButtonBindings() {
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
  public Command getArmCommand(){
    return armCommand;
  }

  public Command getDriveCommand() {
    return driveCommand;
  }

  public Command getWristRotateTestCommand() {
    return new WristRotateTestCommand(m_wristSubsystem, -100);
  }

  public Command getWristPIDCommandTest() {
    return new WristRotatePIDTestCommand(m_wristSubsystem, 0);
  }

  public Command getWristTeleopCommand() {
    return new WristTeleopCommand(m_wristSubsystem, 10);
  }

  public Command getRampClimbTest() {
    return new RampClimbSequence(m_robotDrive);
  }

}
