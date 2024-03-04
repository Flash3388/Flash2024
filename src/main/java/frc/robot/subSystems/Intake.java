package frc.robot.subSystems;

import com.castle.util.dependencies.DependencySupplier;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {
    private CANSparkMax motor;
    private final static double INTAKE_SPEED = 0.4; // 0.85
    private static final double SHOOT_SPEED = 0.5; // 1.0

    private static final double PULL_SPEED = 0.2; // 0.5
    private DigitalInput left;
    private DigitalInput right;


    public Intake(CANSparkMax motor, DigitalInput digitalInputLeft, DigitalInput digitalInputRight){
        this.motor = motor;
        this.left = digitalInputLeft;
        this.right = digitalInputRight;
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void stop(){
        this.motor.stopMotor();
    }

    public void takeIn(){
        this.motor.set(INTAKE_SPEED);
    }

    public void takeOut(){
        this.motor.set(-INTAKE_SPEED);
    }
    public void shoot(){
        this.motor.set(SHOOT_SPEED);
    }
    public boolean isIN(){
        //return !this.left.get();
        return isInLeft() || isInRight();
    }

    public boolean isInRight(){
        return !this.right.get();
    }

    public boolean isInLeft(){
        return !this.left.get();
    }

    public void pullIn() {
        this.motor.set(PULL_SPEED);
    }

    public void print(){
        SmartDashboard.putBoolean("IS IN NOTE", isIN());
        SmartDashboard.putBoolean("IS IN NOTE LEFT", isInLeft());
        SmartDashboard.putBoolean("IS IN NOTE RIGHT", isInRight());
    }
}
