package frc.robot.subSystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterSystem extends Subsystem {
    private CANSparkMax rightEC;
    private CANSparkMax leftEC;
    private RelativeEncoder encoder_1;
    private RelativeEncoder encoder_2;
    private SparkPIDController pidRight;
    private SparkPIDController pidLeft;
    private final double KP_1 = 1e-7; // we need to find the value
    private final double KI_1 = 1e-7;
    private final double KD_1 = 0;
    private final double KF_1 = 0;

    private final double KP_2 = 1e-7; // we need to find the value
    private final double KI_2 = 1e-7;
    private final double KD_2 = 0;

    private final double KF_2 = 0;



    public static final double SPEED_TARGET_SPEAKER = 3000;
    public static final double SPEED_TARGET_AMP = 2000;

    public ShooterSystem(CANSparkMax rightEC, CANSparkMax leftEC){
        this.rightEC = rightEC;
        this.leftEC = leftEC;
        this.encoder_1 = rightEC.getEncoder();
        this.encoder_2 = leftEC.getEncoder();
        this.pidRight = rightEC.getPIDController();
        this.pidLeft = leftEC.getPIDController();
        SmartDashboard.putNumber("KP 1", KP_1);
        SmartDashboard.putNumber("KI 1", KI_1);
        SmartDashboard.putNumber("KD 1", KD_1);
        SmartDashboard.putNumber("KF 1", KD_1);

        SmartDashboard.putNumber("KP 2", KP_2);
        SmartDashboard.putNumber("KI 2", KI_2);
        SmartDashboard.putNumber("KD 2", KD_2);
        SmartDashboard.putNumber("KF 2", KF_2);

        pidRight.setP(KP_1);
        pidRight.setI(KI_1);
        pidRight.setD(KD_1);
        pidRight.setFF(KF_1);

        pidLeft.setP(KP_2);
        pidLeft.setI(KI_2);
        pidLeft.setD(KD_2);
        pidLeft.setFF(KF_2);

        pidRight.setOutputRange(-1,1);
        pidLeft.setOutputRange(-1,1);

        rightEC.setIdleMode(CANSparkBase.IdleMode.kCoast);
        leftEC.setIdleMode(CANSparkBase.IdleMode.kCoast);

    }


    public void shootSpeaker(){
        //double speed = pid.applyAsDoubleapplyAsDouble(getSpeed(),SPEED_TARGET_SPEAKER);
        pidRight.setReference(SPEED_TARGET_SPEAKER, CANSparkBase.ControlType.kVelocity);
        pidLeft.setReference(SPEED_TARGET_SPEAKER, CANSparkBase.ControlType.kVelocity);

        SmartDashboard.putNumber("Speed", getSpeed());
        SmartDashboard.putBoolean("isStopped",false);
        SmartDashboard.putBoolean("got to target Speaker",gotToTarget(SPEED_TARGET_SPEAKER));

    }

    public void shootAmp(){
        pidRight.setReference(SPEED_TARGET_AMP, CANSparkBase.ControlType.kVelocity);
        pidLeft.setReference(SPEED_TARGET_AMP, CANSparkBase.ControlType.kVelocity);

        SmartDashboard.putNumber("Speed", getSpeed());
        SmartDashboard.putBoolean("isStopped",false);
        SmartDashboard.putBoolean("got to target Amp",gotToTarget(SPEED_TARGET_AMP));
    }

    public void reverse(){
        pidRight.setReference(500, CANSparkBase.ControlType.kVelocity);
        pidLeft.setReference(500, CANSparkBase.ControlType.kVelocity);

        SmartDashboard.putNumber("Speed", getSpeed());
        SmartDashboard.putBoolean("isStopped",false);
        SmartDashboard.putBoolean("got to target reverse",gotToTarget(500));

    }

    public double getSpeed(){
        SmartDashboard.putNumber("RPM", (encoder_1.getVelocity() + encoder_2.getVelocity()) / 2);
        return (encoder_1.getVelocity() + encoder_2.getVelocity()) / 2;
    }

    public void resetPID(){
        //pid.reset();
    }

    public void stop(){
        SmartDashboard.putBoolean("isStopped",true);
        SmartDashboard.putBoolean("AMPBool", false);
        SmartDashboard.putBoolean("SpeakerBool", false);
        this.rightEC.stopMotor();
        this.leftEC.stopMotor();
    }
    public boolean gotToTarget(double rpmVal){
        return ExtendedMath.constrained(getSpeed(),rpmVal - 100, rpmVal +100);
    }
}


/*
HELLO TOM MEET THE CODE CAT.
     _
    / )
   / /
  / /               /\
 / /     .-```-.   / ^`-.
 \ \    /       \_/  (|) `o
  \ \  /   .-.   \\ _  ,--'
   \ \/   /   )   \( __^^^
    \   \/    (    )
     \   )     )  /
Hey   ) /__    | (__
     (___)))   (__)))
 */