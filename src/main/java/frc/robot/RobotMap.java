package frc.robot;

import com.flash3388.flashlib.frc.robot.RoboRio;
import com.flash3388.flashlib.hid.HidChannel;

public class RobotMap {
    public static final int ARM_MASTER = 3;
    public static final int ARM_FOLLOW = 3;
    public static final HidChannel ARM_ENCODER = RoboRio.newHidChannel(0);
    public static final double ARM_ANGLE = 0;
}
