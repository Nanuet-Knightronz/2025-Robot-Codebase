package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class MobilityAuto extends Command {
    private final DriveSubsystem drive = DriveSubsystem.getInstance();
    private final ArmSubsystem arm = ArmSubsystem.getInstance();
    
    public MobilityAuto(){
        addRequirements(drive);
        addRequirements(arm);
    }

    @Override
    public void execute() {
        drive.drive(0.3, 0);
        arm.run(() -> {arm.moveArmCommand(-5);});
    }
}
