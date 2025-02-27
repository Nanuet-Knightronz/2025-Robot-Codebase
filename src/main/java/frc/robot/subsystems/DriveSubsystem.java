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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final PhotonCamera camera = new PhotonCamera("photonvision");
  private final PIDController photonPID = new PIDController(0.02, 0, 0.001); 

  public DriveSubsystem() {
    leftLeader = new SparkMax(13, MotorType.kBrushless);
    leftFollower = new SparkMax(12, MotorType.kBrushless);
    rightLeader = new SparkMax(11, MotorType.kBrushless);
    rightFollower = new SparkMax(10, MotorType.kBrushless);

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

  public double getTurnAdjustment() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {

      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw(); // Horizontal offset from center
      int targetID = target.getFiducialId(); // AprilTag ID

      SmartDashboard.putNumber("AprilTag ID", targetID);
      SmartDashboard.putNumber("AprilTag Yaw", yaw);

      return photonPID.calculate(yaw, 0); // PID correction to center the target
    }
    SmartDashboard.putString("AprilTag Status", "No Target Found");
    return 0.0; // No target, don't turn
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
