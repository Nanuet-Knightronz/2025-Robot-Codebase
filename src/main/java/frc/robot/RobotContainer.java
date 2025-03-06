// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
// import frc.robot.commands.PhotonNavCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.auto.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other t00han the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final UsbCamera camera1;
  private final UsbCamera camera2;
 


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driveController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, m_driveController));
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    //CvSource outputStream = CameraServer.putVideo("camera1", 0, 0);
    //CvSource outputStream2 = CameraServer.putVideo("camera2", 0, 0);
    configureBindings();
  }

  private void configureBindings() {

    //move arm up and down with the pov / dpad thing
    m_driveController.povUp().whileTrue(armSubsystem.raiseClimber());
    m_driveController.povDown().whileTrue(armSubsystem.lowerClimber());

    //arm setpoints:
    //DO NOT SET TO POSITIVE VALUES; WILL BREAK ROBOT EXTREMELY QUICKLY 
    m_driveController.a().onTrue(armSubsystem.moveArmCommand(32));
    m_driveController.b().onTrue(armSubsystem.moveArmCommand(22));
    m_driveController.x().onTrue(armSubsystem.moveArmCommand(12));
    m_driveController.y().onTrue(armSubsystem.moveArmCommand(-5));
    //m_driveController.button(7).onTrue(armSubsystem.moveArmCommand(-60));
    m_driveController.button(7).onTrue(armSubsystem.lockArm());
    m_driveController.button(8).onTrue(armSubsystem.unlockArm());
    //m_driveController.button(9).onTrue(armSubsystem.zeroServos());

    //intake stuff
    m_driveController.rightTrigger().whileTrue(new IntakeCommand(intakeSubsystem, 0.5));
    m_driveController.rightBumper().whileTrue(new IntakeCommand(intakeSubsystem, -0.5));

    //photonvision alignment
    m_driveController.leftBumper().whileTrue(new DriveCommand(driveSubsystem, m_driveController));
  }

  public Command getAutonomousCommand() {
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
      return AutoBuilder.followPath(path);
    } 
    catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
