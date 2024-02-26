package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subSystems.*;

public class SystemFactory {
    public static Swerve createSwerveSystem(){
        SwerveModule[] swerveModules = new SwerveModule[4];

        CANSparkMax drive = new CANSparkMax(RobotMap.SWERVE_DRIVE_FL, CANSparkMaxLowLevel.MotorType.kBrushless);
        drive.setInverted(true);
        CANSparkMax steer = new CANSparkMax(RobotMap.SWERVE_STEER_FL, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANCoder absoluteEncoder = new CANCoder(RobotMap.SWERVE_ABSOLUTE_ENCODER_FL);
        swerveModules[0] = new SwerveModule(drive,steer,absoluteEncoder, RobotMap.SWERVE_ABSOLUTE_ENCODER_FL_ZERO_ANGLE);

        drive = new CANSparkMax(RobotMap.SWERVE_DRIVE_FR, CANSparkMaxLowLevel.MotorType.kBrushless);
        drive.setInverted(false);
        steer = new CANSparkMax(RobotMap.SWERVE_STEER_FR, CANSparkMaxLowLevel.MotorType.kBrushless);
        absoluteEncoder = new CANCoder(RobotMap.SWERVE_ABSOLUTE_ENCODER_FR);
        swerveModules[1] = new SwerveModule(drive,steer,absoluteEncoder,RobotMap.SWERVE_ABSOLUTE_ENCODER_FR_ZERO_ANGLE);

        drive = new CANSparkMax(RobotMap.SWERVE_DRIVE_RL, CANSparkMaxLowLevel.MotorType.kBrushless);
        drive.setInverted(true);
        steer = new CANSparkMax(RobotMap.SWERVE_STEER_RL, CANSparkMaxLowLevel.MotorType.kBrushless);
        absoluteEncoder = new CANCoder(RobotMap.SWERVE_ABSOLUTE_ENCODER_RL);
        swerveModules[2] = new SwerveModule(drive,steer,absoluteEncoder,RobotMap.SWERVE_ABSOLUTE_ENCODER_RL_ZERO_ANGLE);

        drive = new CANSparkMax(RobotMap.SWERVE_DRIVE_RR, CANSparkMaxLowLevel.MotorType.kBrushless);
        drive.setInverted(false);
        steer = new CANSparkMax(RobotMap.SWERVE_STEER_RR, CANSparkMaxLowLevel.MotorType.kBrushless);
        absoluteEncoder = new CANCoder(RobotMap.SWERVE_ABSOLUTE_ENCODER_RR);
        swerveModules[3] = new SwerveModule(drive,steer,absoluteEncoder,RobotMap.SWERVE_ABSOLUTE_ENCODER_RR_ZERO_ANGLE);

        WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON);

        return new Swerve(swerveModules, gyro);
    }
    public static ShooterSystem createShooter() {
        CANSparkMax master = new CANSparkMax(RobotMap.SHOOTER_MASTER, CANSparkLowLevel.MotorType.kBrushless);
        CANSparkMax follower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);
        return new ShooterSystem(master, follower);
    }

    public static Intake createIntake() {
            CANSparkMax canSparkMax = new CANSparkMax(RobotMap.INTAKE, CANSparkLowLevel.MotorType.kBrushless);
            Intake intake = new Intake(canSparkMax, new DigitalInput(6), new DigitalInput(1));
            return intake;
        }


    public static Arm createArm(){
        CANSparkMax master = new CANSparkMax(RobotMap.ARM_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax follow = new CANSparkMax(RobotMap.ARM_FOLLOW, CANSparkMaxLowLevel.MotorType.kBrushless);
        DutyCycleEncoder encoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER);
        return new Arm(master, follow, encoder);
    }

    public static Climb createClimb(){
        CANSparkMax motor = new CANSparkMax(RobotMap.CLIMB_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        return new Climb(motor);
    }

}
