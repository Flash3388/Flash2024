package frc.robot.Subsystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSystem extends Subsystem {
    private CANSparkMax master;
    private CANSparkMax follower;
    private static final double SPEED_LIMIT = 0.3;

    public ShooterSystem(CANSparkMax master, CANSparkMax follower){
        this.master = master;
        this.follower = follower;
        this.follower.follow(master);
        this.follower.setInverted(true); // Not sure if this does what I think it does, which is reversing the input given to it by the master.
    }

    public void move(double speed){
        speed = ExtendedMath.constrain(speed, -SPEED_LIMIT, SPEED_LIMIT);
        this.master.set(speed); // Need to set up PID for speed.
    }

    public void stop(){
        this.master.stopMotor();
    }
}
