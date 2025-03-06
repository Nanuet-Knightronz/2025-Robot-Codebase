package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.lang.reflect.Field;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;


public class DriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final PhotonCamera camera;
  private final PIDController visionPID;
  private final AHRS navX;
  private final DifferentialDriveOdometry odometry;
  private final Field2d field;


  private static final double kP = 0.02; // Proportional gain (Adjust based on testing)
  private static final double kI = 0.0;  // Integral gain (Usually 0 for vision)
  private static final double kD = 0.001; // Derivative gain (Smooths motion)

  private static final DriveSubsystem instance = new DriveSubsystem();

  public static DriveSubsystem getInstance(){
    return instance;
  }

  public DriveSubsystem() {

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e){
      e.printStackTrace();
    }


    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );

    leftLeader = new SparkMax(13, MotorType.kBrushless);
    leftFollower = new SparkMax(12, MotorType.kBrushless);
    leftEncoder = leftLeader.getEncoder();
    rightLeader = new SparkMax(11, MotorType.kBrushless);
    rightFollower = new SparkMax(10, MotorType.kBrushless);
    rightEncoder = rightLeader.getEncoder();

    camera = new PhotonCamera("USB_Camera"); // Change to actual camera name

    visionPID = new PIDController(kP, kI, kD);
    visionPID.setTolerance(1.0); // Degrees of yaw tolerance before stopping corrections

    SparkMaxConfig globalConfig = new SparkMaxConfig();
      globalConfig.smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);

    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.apply(globalConfig).inverted(true);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.apply(globalConfig).follow(leftLeader);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.apply(globalConfig).follow(rightLeader);

    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    navX = new AHRS(NavXComType.kMXP_SPI);
    navX.reset();

    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    field = new Field2d();


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
  
  public Pose2d getPose() {
    // Return the current pose (e.g., from your odometry system)
    //return new Pose2d(0, 0, new Rotation2d(0)); // Replace with actual pose
    return odometry.getPoseMeters();
}

public void resetPose(Pose2d pose) {
    // Reset the robot pose (e.g., to a known position)
    // Example: odometry.resetPosition(pose, new Rotation2d());
    odometry.resetPosition(navX.getRotation2d(), null, pose);
}

public ChassisSpeeds getRobotRelativeSpeeds() {
    // Return the current robot speeds (e.g., from your odometry system or joystick input)
    return new ChassisSpeeds(0, 0, 0); // Replace with actual speeds
}

public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert the robot-relative speeds into individual motor outputs and set them
    double leftSpeed = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond;
    double rightSpeed = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond;

    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);
}

  @Override
  public void periodic() {
    navX.getAngle();
    SmartDashboard.putNumber("navX Angle", navX.getAngle());
    SmartDashboard.putData("Field", field);
    odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());

  }

}
