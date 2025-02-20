package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax m_LeftRaise;
    private SparkMax m_RightRaise;
    private final RelativeEncoder armEncoder;
    private final PIDController armPID;

    private double targetPosition;

    public ArmSubsystem() {

        m_LeftRaise = new SparkMax(14, MotorType.kBrushless);
        m_RightRaise = new SparkMax(15, MotorType.kBrushless);

        armEncoder = m_LeftRaise.getAlternateEncoder();

        armPID = new PIDController(Constants.Arm.armkP, Constants.Arm.armkI, Constants.Arm.armkD);
        armPID.setTolerance(1.0); // how much stuff it can be off by

        targetPosition = armEncoder.getPosition();
    }

    public void moveArmToPosition(double position) {
      targetPosition = position;
    }

    @Override
    public void periodic() {
      double currentPosition = armEncoder.getPosition();
        
        // ✅ Calculate the PID output based on the error
        double pidOutput = armPID.calculate(currentPosition, targetPosition);

        // ✅ Limit the motor speed to prevent sudden jumps
        double motorOutput = MathUtil.clamp(pidOutput, -.3, .3);

        // ✅ Apply output to motor
        m_LeftRaise.set(motorOutput);

        SmartDashboard.putNumber("Arm Position", currentPosition);
    }

    public Command moveArmCommand(double position) {
      return runEnd(() -> moveArmToPosition(position), () -> moveArmToPosition(targetPosition));
    }

    public Command raiseClimber() {
        return runEnd(() -> {raise();}, () -> {idleArm();});
    }

    public Command lowerClimber() {
        return runEnd(() -> {lower();}, () -> {idleArm();});
    }

    public void raise() {
        m_LeftRaise.set(-.3);
        m_RightRaise.set(.3);
    }

    public void lower() {
        m_LeftRaise.set(.3);
        m_RightRaise.set(-.3);
    }

    public void idleArm() {
        m_LeftRaise.set(0);
        m_RightRaise.set(0);
    }

    public void resetEncoder() {
      armEncoder.setPosition(0);
  }
}