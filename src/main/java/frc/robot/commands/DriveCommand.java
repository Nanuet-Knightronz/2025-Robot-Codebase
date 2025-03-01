// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController joystick;
  
  public DriveCommand(DriveSubsystem driveSubsystem, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.joystick = joystick;
    addRequirements(driveSubsystem); // Declare the subsystem used
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -joystick.getLeftY() * Math.abs(joystick.getLeftY());  // Invert Y for forward movement
    double rotation = joystick.getRightX() * Math.abs(joystick.getRightX()); // X-axis controls rotation

    driveSubsystem.drive(forward, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}