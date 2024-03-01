package frc.robot.subSystems;

import com.castle.util.dependencies.DependencySupplier;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends Subsystem {
    private CANSparkMax motor;
    private final static double INTAKE_SPEED = 0.85;
    private static final double SHOOT_SPEED = 1;

    private static final double PULL_SPEED = 0.5;
    private DigitalInput in;
    private DigitalInput agam;

    public static boolean IS_IN = false;

    public Intake(CANSparkMax motor,  DigitalInput digitalInput, DigitalInput agam){
        this.motor = motor;
        this.in = digitalInput;
        this.agam = agam;
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
        return !this.in.get();
    }

    public boolean isInAgam(){
        return this.agam.get();
    }

    public void pullIn() {
        this.motor.set(PULL_SPEED);
    }
}
