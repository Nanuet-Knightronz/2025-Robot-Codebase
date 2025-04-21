package frc.robot.subsystems;

import frc.robot.SwerveMod;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Swerve extends SubsystemBase {
    private PoseEstimator s_PoseEstimator = new PoseEstimator();
    private PhotonCamera camera;
    private ProfiledPIDController rotationController;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveMod[] mSwerveMods;
    public PigeonIMU gyro;
    public final AHRS navx;
    public RobotConfig config;
    private Field2d field = new Field2d();

    private final PIDController visionPID;
    private PhotonPoseEstimator photonEstimator;

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
     

    //photonvision PID
    private static final double kvP = .02;
    private static final double kvI = 0.0;  // Integral gain (Usually 0 for vision)
    private static final double kvD = 0.001; // Derivative gain (Smooths motion)

    public Swerve(PoseEstimator s_PoseEstimator) {

        //actually do photon stuff
        camera = new PhotonCamera("USB_Camera");

        photonEstimator = 
            new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);

        visionPID = new PIDController(kvP, kvI, kvD);
        visionPID.setTolerance(1.0);

        this.s_PoseEstimator = s_PoseEstimator;
        

        config = new RobotConfig(
          Constants.AutoConstants.ROBOT_MASS_KG,
          Constants.AutoConstants.ROBOT_MOI,
          Constants.AutoConstants.moduleConfig,
          Constants.Swerve.trackWidth);

        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.zeroYaw();

        mSwerveMods = new SwerveMod[] {
            new SwerveMod(0, Constants.Swerve.Mod0.constants),
            new SwerveMod(1, Constants.Swerve.Mod1.constants),
            new SwerveMod(2, Constants.Swerve.Mod2.constants),
            new SwerveMod(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    Constants.AutoConstants.translationPID, // Translation PID constants
                    Constants.AutoConstants.rotationPID // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);

        // Initialize PhotonVision camera and rotation controller
        rotationController = new ProfiledPIDController(1.0, 0.0, 0.0, 
            new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation
                                )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveMod mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveMod mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose); // First used to be getHeading()
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveMod mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveMod mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(navx.getYaw());
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void resetModulesToAbsolute(){
        for(SwerveMod mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getVisionYaw() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0; //no target = nothing
    }

    public double getVisionCorrection() {
    double yaw = getVisionYaw();
    if (Math.abs(yaw) < visionPID.getPositionTolerance()) {
      return 0; // Don't correct if within tolerance
    }
    return visionPID.calculate(-yaw, 0); // PID correction
  }
  

    @Override
    public void periodic(){
        
        swerveOdometry.update(getGyroYaw(), getModulePositions()); 
        //s_PoseEstimator.updateSwerve(getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Get Gyro", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Get Heading", getHeading().getDegrees());
        for(SwerveMod mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
    }
}