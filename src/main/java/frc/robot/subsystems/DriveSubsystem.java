// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  public DriveSubsystem() {
    leftLeader = new SparkMax(1, MotorType.kBrushless);
    leftFollower = new SparkMax(2, MotorType.kBrushless);
    rightLeader = new SparkMax(3, MotorType.kBrushless);
    rightFollower = new SparkMax(4, MotorType.kBrushless);

    // Set up configurations as in the original code
    SparkMaxConfig globalConfig = new SparkMaxConfig();
      globalConfig.smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
      rightLeaderConfig.apply(globalConfig)
      .inverted(true);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
      leftFollowerConfig.apply(globalConfig)
      .follow(leftLeader);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
      rightFollowerConfig.apply(globalConfig)
      .follow(rightLeader);

    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void drive(double forward, double rotation) {
    leftLeader.set(forward + rotation);
    rightLeader.set(forward - rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
