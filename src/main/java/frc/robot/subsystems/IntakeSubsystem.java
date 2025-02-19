// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final Spark m_TopRoller;
  private final Spark m_BottomRoller;

  public IntakeSubsystem() {
    m_TopRoller = new Spark(Constants.Intake.kIntakeMotorTopPWMId);
    m_BottomRoller = new Spark(Constants.Intake.kIntakeMotorBottomPWMId);
  }

  public void moveRollers(double velocity) {
    m_TopRoller.set(velocity);
    m_BottomRoller.set(-velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
