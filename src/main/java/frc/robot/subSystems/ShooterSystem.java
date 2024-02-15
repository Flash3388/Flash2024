package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterSystem extends Subsystem {
    private CANSparkMax rightEC;
    private CANSparkMax leftEC;
    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;
    private SparkPIDController pidRight;
    private SparkPIDController pidLeft;
    private  double KP_RIGHT = 0.0000065; // we need to find the value
    private  double KI_RIGHT = 0.0000002;
    private  double KD_RIGHT = 0;
    private  double KF_RIGHT = 0;

    private  double KP_LEFT = 0.0000065; // we need to find the value
    private  double KI_LEFT = 0.0000002;
    private  double KD_LEFT = 0;
    private  double KF_LEFT = 0;



    public static final double SPEED_TARGET_SPEAKER = 4000;
    public static final double SPEED_TARGET_AMP = 500;
    public ShooterSystem(CANSparkMax rightEC, CANSparkMax leftEC){
        this.rightEC = rightEC;
        this.leftEC = leftEC;

        this.rightEncoder = rightEC.getEncoder();
        this.leftEncoder = leftEC.getEncoder();

        this.pidRight = rightEC.getPIDController();
        this.pidLeft = leftEC.getPIDController();

        SmartDashboard.putNumber("KP RIGHT", KP_RIGHT);
        SmartDashboard.putNumber("KI RIGHT", KI_RIGHT);
        SmartDashboard.putNumber("KD RIGHT", KD_RIGHT);
        SmartDashboard.putNumber("KF RIGHT", KD_RIGHT);

        SmartDashboard.putNumber("KP LEFT", KP_LEFT);
        SmartDashboard.putNumber("KI LEFT", KI_LEFT);
        SmartDashboard.putNumber("KD LEFT", KD_LEFT);
        SmartDashboard.putNumber("KF LEFT", KF_LEFT);

        SmartDashboard.putNumber("set point velocity", 0);

        pidRight.setP(KP_RIGHT);
        pidRight.setI(KI_RIGHT);
        pidRight.setD(KD_RIGHT);
        pidRight.setFF(KF_RIGHT);

        pidLeft.setP(KP_LEFT);
        pidLeft.setI(KI_LEFT);
        pidLeft.setD(KD_LEFT);
        pidLeft.setFF(KF_LEFT);

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
        SmartDashboard.putNumber("RPM", (rightEncoder.getVelocity() + leftEncoder.getVelocity()) / 2);
        return (rightEncoder.getVelocity() + leftEncoder.getVelocity()) / 2;
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
        final double SPEED_ERROR = 100;
        return ExtendedMath.constrained(getSpeed(),rpmVal - SPEED_ERROR, rpmVal + SPEED_ERROR);
    }

    public void print(){
        SmartDashboard.putNumber("right velocity", rightEncoder.getVelocity());
        SmartDashboard.putNumber("left velocity", leftEncoder.getVelocity());

    }

    public void changePidValues(){
        double pR = SmartDashboard.getNumber("KP RIGHT", KP_RIGHT);
        double iR = SmartDashboard.getNumber("KI RIGHT", KI_RIGHT);
        double dR = SmartDashboard.getNumber("KD RIGHT", KD_RIGHT);


        if((pR != KP_RIGHT)) { pidRight.setP(pR); KP_RIGHT = pR; }
        if((iR != KI_RIGHT)) { pidRight.setI(iR); KI_RIGHT = iR; }
        if((dR != KD_RIGHT)) { pidRight.setD(dR); KD_RIGHT = dR; }


        double pL = SmartDashboard.getNumber("KP LEFT", KP_LEFT);
        double iL = SmartDashboard.getNumber("KI LEFT", KI_LEFT);
        double dL = SmartDashboard.getNumber("KD LEFT", KD_LEFT);


        if((pL != KP_LEFT)) { pidLeft.setP(pL); KP_LEFT = pL; }
        if((iL != KI_LEFT)) { pidLeft.setI(iL); KI_LEFT = iL; }
        if((dL != KD_LEFT)) { pidLeft.setD(dL); KD_LEFT = dL; }


    }

    public void setVelocity(double velocity){
        pidRight.setReference(velocity, CANSparkBase.ControlType.kVelocity);
        pidLeft.setReference(velocity, CANSparkBase.ControlType.kVelocity);
    }

    public void resetI(){
        pidRight.setIAccum(0);
        pidLeft.setIAccum(0);
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