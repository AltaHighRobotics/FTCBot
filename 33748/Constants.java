package org.firstinspires.ftc.teamcode;


public class Constants {
    /* 
    This file holds all the constants for the robot
    This makes it easy to change constants
    */
    public class LauncherConstants {
        
        // RPMs for both shooting distances
        public static final int kFarRevs = 5000;
        public static final int kNearRevs = 5000;
        public static final int kIdle = 1000;
        
        // PID
        public static final double kP = 0.0005;
        public static final double kI = 0.000001;
        public static final double kD = 0.0001;
        
    }
    
    public class AprilTagConstants {
        
        // Acceptable error for the alignment
        public static final double kAcceptableError = 0.05;
        
        // PID
        public static final double kP = 0.0005;
        public static final double kI = 0.000001;
        public static final double kD = 0.0001; 
        
    }
    
}
