package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subSystems.Intake;

import frc.robot.subSystems.ShooterSystem;

public class SystemFactory {
    public static ShooterSystem createShooter() {
        CANSparkMax master = new CANSparkMax(RobotMap.SHOOTER_MASTER, CANSparkLowLevel.MotorType.kBrushless);
        CANSparkMax follower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);
        return new ShooterSystem(master, follower);
    }

    public static Intake createIntake() {
            CANSparkMax canSparkMax = new CANSparkMax(RobotMap.INTAKE, CANSparkLowLevel.MotorType.kBrushless);
            Intake intake = new Intake(canSparkMax, new DigitalInput(6));
            return intake;
        }

    }


