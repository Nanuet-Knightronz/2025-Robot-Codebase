package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax m_LeftRaise;
    private SparkMax m_RightRaise;
    private final AbsoluteEncoder armEncoder;
    private final RelativeEncoder relEncoder;
    private final PIDController armPID;
    private double armPosition;
    private double targetPosition;
    private double armPIDResult;
    public ArmSubsystem() {

        m_LeftRaise = new SparkMax(14, MotorType.kBrushless);
        m_RightRaise = new SparkMax(15, MotorType.kBrushless);
    
        armEncoder = m_LeftRaise.getAbsoluteEncoder();
        relEncoder = m_LeftRaise.getEncoder();

        armPID = new PIDController(Constants.Arm.armkP, Constants.Arm.armkI, Constants.Arm.armkD);
        //armPID.setTolerance(1.0); // how much stuff it can be off by
    }

    @Override
    public void periodic() {
        armPosition = relEncoder.getPosition();
        armPIDResult = armPID.calculate(armPosition, targetPosition);
        SmartDashboard.putNumber("Encoder", relEncoder.getPosition());
        SmartDashboard.putNumber("PID Result", armPIDResult);
        SmartDashboard.putNumber("Setpoint", targetPosition);
    }

    public Command moveArmCommand(double position) {

      return run(()-> {
        //double armPosition = armEncoder.getPosition();
        //no flipping pls
        //double target = MathUtil.clamp(position, 0, 35);

        //double motorOutput = MathUtil.clamp(armPIDResult, -.3, .3);

        this.targetPosition = position;
        m_LeftRaise.set(MathUtil.clamp(armPIDResult,-0.5,0.5));
        m_RightRaise.set(-MathUtil.clamp(armPIDResult,-0.5,0.5));
      });
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
       relEncoder.setPosition(0);
  }
}