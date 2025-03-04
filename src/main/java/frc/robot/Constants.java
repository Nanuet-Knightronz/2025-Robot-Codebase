package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

public class Constants {
    public static class Intake {
        public static final int kIntakeMotorTopPWMId = 1;
        public static final int kIntakeMotorBottomPWMId = 0;
    }
    public static class Arm {
        public static final int kArmMotorLeftPWMId = 14;
        public static final int kArmMotorRightPWMId = 15;
        public static final double armkP = 0.05;  
        public static final double armkI = 0.0;   
        public static final double armkD = 0.002; 

    }
    public static class Drivetrain {
        public static final double kMaxSpeed = .9;
        public static final int leftLeader_SPARKMAX = 13;
        public static final int leftFollower_SPARKMAX = 12;
        public static final int rightLeader_SPARKMAX = 11;
        public static final int rightFollower_SPARKMAX = 10;
    }
}
