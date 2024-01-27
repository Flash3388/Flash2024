package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import subSystems.Arm;

public class SystemFactory {
    public static Arm createArm(){
        CANSparkMax master = new CANSparkMax(RobotMap.ARM_MASTER, CANSparkMaxLowLevel.MotorType.kBrushed);
        CANSparkMax follow = new CANSparkMax(RobotMap.ARM_FOLLOW, CANSparkMaxLowLevel.MotorType.kBrushless);
        DutyCycleEncoder encoder = new DutyCycleEncoder((DutyCycle) RobotMap.ARM_ENCODER);
        double angle2Target = RobotMap.ARM_ANGLE;
        return new Arm(angle2Target, master, follow, encoder);
    }
}
