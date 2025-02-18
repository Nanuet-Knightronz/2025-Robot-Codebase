package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax m_LeftRaise;
    private SparkMax m_RightRaise;

    public ArmSubsystem() {
        m_LeftRaise = new SparkMax(Constants.Arm.kArmMotorLeftPWMId, MotorType.kBrushless);
        m_RightRaise = new SparkMax(Constants.Arm.kArmMotorRightPWMId, MotorType.kBrushless);

        
    }

    @Override
    public void periodic() {
    }

    public Command raiseClimber() {
        return runEnd(() -> {raise();}, () -> {idleArm();});
    }

    public Command lowerClimber() {
        return runEnd(() -> {lower();}, () -> {idleArm();});
    }

    public void raise() {
        m_LeftRaise.set(-1);
        m_RightRaise.set(1);
    }

    public void lower() {
        m_LeftRaise.set(1);
        m_RightRaise.set(-1);
    }

    public void idleArm() {
        m_LeftRaise.set(0);
        m_RightRaise.set(0);
    }
}