// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import frc.robot.Constants.VisionPID;
import frc.robot.Constants.VisionConstants;

public class PhotonAlignPoseCommand extends Command {

  private PoseEstimator poseEstimator;
  private Swerve s_Swerve;
  private Vision vision;

    private final PIDController strafeController = new PIDController(VisionPID.kvP, VisionPID.kvI, VisionPID.kvD);
    private final PIDController forwardController = new PIDController(VisionPID.kvP, VisionPID.kvI, VisionPID.kvD);
    private final PIDController rotationController = new PIDController(VisionPID.kvP, VisionPID.kvI, VisionPID.kvD);

    private final Supplier<Pose2d> poseProvider;

    private static final Transform3d tagToPose =
      new Transform3d(
        new Translation3d(1.5, 0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI)
      );
    

  public PhotonAlignPoseCommand(
          Swerve s_Swerve,  
          PoseEstimator poseEstimator, 
          Supplier<Pose2d> poseProvider, Vision vision) { 
    this.s_Swerve = s_Swerve;
    this.poseEstimator = poseEstimator;
    this.vision = vision;
    this.poseProvider = poseProvider;

    rotationController.setTolerance(Units.degreesToRadians(3));
    strafeController.setTolerance(.3);
    forwardController.setTolerance(.3);

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var robotPose = poseProvider.get();
     rotationController.reset();
     strafeController.reset();
     forwardController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose2d = poseProvider.get();
    var robotPose = new Pose3d(
      robotPose2d.getX(),
      robotPose2d.getY(),
      0.0, 
      new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians())
    );

    var photonResult = vision.getCamera().getAllUnreadResults();

    //find camera pose
    Pose3d cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_CAMERA);

    //find the target pose and turn it into pose2d
    PhotonPipelineResult result = vision.getCamera().getLatestResult();
    if (!result.hasTargets()) {
      return; // Skip if no targets found
    }
    PhotonTrackedTarget target = result.getBestTarget();
      var camToTarget = target.getBestCameraToTarget();
      var targetPose = cameraPose.transformBy(camToTarget);
    var finalPose = targetPose.transformBy(tagToPose).toPose2d();

    //plug finalPose into pid to get there
    double rotationOutput = rotationController.calculate(
        robotPose2d.getRotation().getRadians(),   
        finalPose.getRotation().getRadians()
    );
    double strafeOutput = strafeController.calculate(robotPose2d.getY(), finalPose.getY());
    double forwardOutput = forwardController.calculate(robotPose2d.getX(), finalPose.getX());

    //drive based off of controllers
    s_Swerve.drive(new Translation2d(forwardOutput, strafeOutput), rotationOutput, true, false);
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
