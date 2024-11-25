// package frc.robot.commands;

// import edu.wpi.first.math.filter.SlewRateLimiter;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveSubsystem;


// import java.util.function.Supplier;

// public class SwerveJoystickCmd extends CommandBase {

//     private final SwerveSubsystem swerveSubsystem;
//     private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
//     private final Supplier<Boolean> fieldOrientedFunction;
//     private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

//     public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
//         Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction,
//         Supplier<Boolean> fieldOrientedFunction) {
//         this.swerveSubsystem = swerveSubsystem;
//         this.xSpdFunction = xSpdFunction;
//         this.ySpdFunction = ySpdFunction;
//         this.turningSpdFunction = turnSpdFunction;
//         this.fieldOrientedFunction = fieldOrientedFunction;
//         this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
//         this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
//         this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
//         addRequirements(swerveSubsystem);
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override
//     public void execute() {
//         //realtime joystick stuff
//         double xSpeed = xSpdFunction.get();
//         double ySpeed = ySpdFunction.get();
//         double turningSpeed = turningSpdFunction.get();

//         //hopefully this deadband works
//         xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadBand ? xSpeed : 0.0;
//         ySpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadBand ? ySpeed : 0.0;
//         turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.kDeadBand ? turningSpeed : 0.0;

//         //de-rocketfy the input
//         xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//         ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//         turningSpeed = turningLimiter.calculate(turningSpeed)
//                 * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

//         //Idk how fast this thing is supposed to go
//         ChassisSpeeds chassisSpeeds;
//         if (fieldOrientedFunction.get()) {
//             chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                     xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
//         } else {
//             //have you asked how the robot feeling today?
//             chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
//         }
//         //turning robot speed to 4 swerve module states?
//         SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

//         //control the spinny things
//         swerveSubsystem.setModuleStates(moduleStates);
//     }


//     @Override
//     public void end(boolean interrupted) {
//         swerveSubsystem.stopModules();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
