package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax m_LeftRaise;
    private SparkMax m_RightRaise;
    private final RelativeEncoder relEncoder;
    private final PIDController armPID;
    private double armPosition;
    private double targetPosition;
    private double armPIDResult;
    private DigitalInput armLimit;
    private Servo leftStop;
    private Servo rightStop;
    private Boolean isRatchetEngaged;

    private static final ArmSubsystem instance = new ArmSubsystem();

    public static ArmSubsystem getInstance(){
        return instance;
    }

    public ArmSubsystem() {
        try{
            m_LeftRaise = new SparkMax(14, MotorType.kBrushless);
            m_RightRaise = new SparkMax(15, MotorType.kBrushless);
        
        }catch(Exception e){}

        leftStop = new Servo(2);
        rightStop = new Servo(3);
    
        relEncoder = m_LeftRaise.getEncoder();
        armLimit = new DigitalInput(0);
        armPID = new PIDController(Constants.Arm.armkP, Constants.Arm.armkI, Constants.Arm.armkD);
        isRatchetEngaged = false;
        //armPID.setTolerance(1.0); // how much stuff it can be off by
    }

    @Override
    public void periodic() {
        armPosition = MathUtil.clamp(relEncoder.getPosition(),-5,32);
        armPIDResult = MathUtil.clamp(armPID.calculate(armPosition, targetPosition),-0.5,0.5);
        SmartDashboard.putNumber("Encoder", relEncoder.getPosition());
        SmartDashboard.putNumber("PID Result", armPIDResult);
        SmartDashboard.putNumber("Setpoint", targetPosition);
        SmartDashboard.putBoolean("LowerLimitSwitch", armLimit.get());
        SmartDashboard.getBoolean("Is Ratchet Engaged?", isRatchetEngaged);
    }

    public Command moveArmCommand(double position) {

      return run(()-> {
        //double armPosition = armEncoder.getPosition();
        //no flipping pls
        //double target = MathUtil.clamp(position, 0, 35);

        //double motorOutput = MathUtil.clamp(armPIDResult, -.3, .3);

        this.targetPosition = MathUtil.clamp(position, -5, 32);
        m_LeftRaise.set(MathUtil.clamp(armPIDResult, -.5, .5));
        m_RightRaise.set(-MathUtil.clamp(armPIDResult,-.5, .5));
      });
    }

    public Command raiseClimber() {
        return runEnd(() -> {raise();}, () -> {idleArm();});
    }

    public Command lowerClimber() {
        if(!armLimit.get());{
            return runEnd(() -> {lower();}, () -> {idleArm();});
        }
    }

    public Command lockArm() {
        return run(()->{lockAxle();});
    }

    public Command unlockArm(){
        return run(()->{unlockAxle();});
    }

    public Command zeroServos(){
        return run(() -> {zeroAxleServos();});
    }

    public void raise() {
        m_LeftRaise.set(-.3);
        m_RightRaise.set(.3);
        targetPosition = MathUtil.clamp(armPosition, -38, 32);
    }

    public void lower() {
        m_LeftRaise.set(.3);
        m_RightRaise.set(-.3);
        targetPosition = MathUtil.clamp(armPosition,-38,32);
    }

    public void idleArm() {
        m_LeftRaise.set(0);
        m_RightRaise.set(0);
        moveArmCommand(relEncoder.getPosition());
    }

    public void lockAxle()  {
        leftStop.setAngle(-80);
        rightStop.setAngle(80);
        isRatchetEngaged = true;
    }

    public void unlockAxle(){
        leftStop.setAngle(80);
        rightStop.setAngle(-80);
        isRatchetEngaged = false;
    }

    public void zeroAxleServos(){
        leftStop.setAngle(0);
        rightStop.setAngle(0);
    }

    public void resetEncoder() {
       relEncoder.setPosition(0);
  }
}