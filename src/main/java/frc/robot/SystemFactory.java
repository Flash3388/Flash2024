package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Subsystems.ShooterSystem;

public class SystemFactory {
    public static ShooterSystem createShooter(){
        CANSparkMax master = new CANSparkMax(RobotMap.SHOOTER_MASTER, CANSparkLowLevel.MotorType.kBrushless);
        CANSparkMax follower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);
        return new ShooterSystem(master, follower);
    }
}
