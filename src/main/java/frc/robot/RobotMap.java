package frc.robot;

import com.flash3388.flashlib.frc.robot.RoboRio;
import com.flash3388.flashlib.hid.HidChannel;

public class RobotMap {
    public static final int ARM_MASTER = 15;
    public static final int ARM_FOLLOW = 16;
    public static final int ARM_ENCODER = 9;
    public static final int CLIMB_MOTOR = 18;
    public static final int PIGEON = 9;

    public static final int SWERVE_DRIVE_FL = 51;
    public static final int SWERVE_DRIVE_FR = 41;
    public static final int SWERVE_DRIVE_RL = 61;
    public static final int SWERVE_DRIVE_RR = 31;
    public static final int SWERVE_STEER_FL = 52;
    public static final int SWERVE_STEER_FR = 42;
    public static final int SWERVE_STEER_RL = 62;
    public static final int SWERVE_STEER_RR = 32;

    public static final int SWERVE_ABSOLUTE_ENCODER_FL = 5;
    public static final int SWERVE_ABSOLUTE_ENCODER_FR = 4;
    public static final int SWERVE_ABSOLUTE_ENCODER_RL = 6;
    public static final int SWERVE_ABSOLUTE_ENCODER_RR = 3;
    public static final double SWERVE_ABSOLUTE_ENCODER_FL_ZERO_ANGLE = 271.669 ;//275.8
    public static final double SWERVE_ABSOLUTE_ENCODER_FR_ZERO_ANGLE = 42.7148 +180 ;// 60.105  60.996
    public static final double SWERVE_ABSOLUTE_ENCODER_RL_ZERO_ANGLE = 32.695 + 180; //216.47
    public static final double SWERVE_ABSOLUTE_ENCODER_RR_ZERO_ANGLE = 12.744 + 180 ; //190.458
    public static final int INTAKE = 13;
    public static final HidChannel XBOX_SYSTEMS = RoboRio.newHidChannel(0);
    public static final HidChannel XBOX_DRIVER = RoboRio.newHidChannel(1);
    public static final int SHOOTER_MASTER = 12;
    public static final int SHOOTER_FOLLOWER = 14;
}
