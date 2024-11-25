// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    double maximumSpeed = 3.5;

    SwerveDrive swerveDrive;

    /** Creates a new SwerveSubsystem. */
    public SwerveSubsystem() {
        // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"tabiSwerve");
        try
        {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        // swerveDrive.setChassisDiscretization(true, 0.02);
        setupPathplanner();
    }

    public void setupPathplanner(){
        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(2.5*0),
                new PIDConstants(0.5*0),
                maximumSpeed,
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig(false, false));

        BooleanSupplier shouldFlipPath = () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followe
            // d to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::setChassisSpeeds,
                holonomicPathFollowerConfig,
                shouldFlipPath,
                this
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                    angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                    true,
                    false);
        });
    }

    public Rotation2d getHeading(){
        return swerveDrive.getGyro().getRotation3d().toRotation2d();
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getChassisSpeeds()
    {
        return swerveDrive.getRobotVelocity();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void stopModules() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopModules'");
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setModuleStates'");
    }

    public Rotation2d getRotation2d() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotation2d'");
    }
}