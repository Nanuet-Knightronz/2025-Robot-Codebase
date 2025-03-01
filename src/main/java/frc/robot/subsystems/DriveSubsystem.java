package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  private final PhotonCamera camera;
  private final PIDController visionPID;

  private static final double kP = 0.02; // Proportional gain (Adjust based on testing)
  private static final double kI = 0.0;  // Integral gain (Usually 0 for vision)
  private static final double kD = 0.001; // Derivative gain (Smooths motion)

  public DriveSubsystem() {
    leftLeader = new SparkMax(13, MotorType.kBrushless);
    leftFollower = new SparkMax(12, MotorType.kBrushless);
    rightLeader = new SparkMax(11, MotorType.kBrushless);
    rightFollower = new SparkMax(10, MotorType.kBrushless);

    camera = new PhotonCamera("USB_Camera"); // Change to actual camera name

    visionPID = new PIDController(kP, kI, kD);
    visionPID.setTolerance(1.0); // Degrees of yaw tolerance before stopping corrections

    SparkMaxConfig globalConfig = new SparkMaxConfig();
      globalConfig.smartCurrentLimit(40)
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

  public double getVisionYaw() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget().getYaw();
    }
    return 0; // No target found
  }

  public double getVisionCorrection() {
    double yaw = getVisionYaw();
    if (Math.abs(yaw) < visionPID.getPositionTolerance()) {
      return 0; // Don't correct if within tolerance
    }
    return visionPID.calculate(-yaw, 0); // PID correction
  }

  @Override
  public void periodic() {}
}
