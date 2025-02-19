// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
    private final IntakeSubsystem m_IntakeSubsystem;
    private final double velocity;
  
    public IntakeCommand(IntakeSubsystem intakeSubsystem, double velocity) {
      this.m_IntakeSubsystem = intakeSubsystem;
      this.velocity = velocity;
    addRequirements(intakeSubsystem);
  }

  public void initialize() {
    m_IntakeSubsystem.moveRollers(velocity); // Start intake
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.moveRollers(velocity); // Ensure intake continues running
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
