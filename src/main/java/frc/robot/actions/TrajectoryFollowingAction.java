package frc.robot.actions;

import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subSystems.Swerve;

import java.util.function.Supplier;

public class TrajectoryFollowingAction extends ActionBase {

    private final edu.wpi.first.wpilibj.Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final Swerve swerve;
    private PidController pid;

    public TrajectoryFollowingAction(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
                                     PIDController xController, PIDController yController, ProfiledPIDController thetaController,
                                     Supplier<Rotation2d> desiredRotation,
                                     Swerve swerve) {

        m_trajectory = trajectory;
        m_pose = pose; //?
        m_kinematics = kinematics; //?

        m_controller = new HolonomicDriveController( //should I create this in the TrajectoryExample class?
                xController,
                yController,
                thetaController);

        m_desiredRotation = desiredRotation;

        this.swerve = swerve;

        requires(swerve);
    }

    @Override
    public void initialize(ActionControl control) {
        m_timer.reset();
        swerve.resetAllEncoders();
        swerve.setHeadingDegrees();
        m_timer.start();
    }

    @Override
    public void execute(ActionControl control) {
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);

        var targetChassisSpeeds = // how to create the current pose and how to create the goal? (to add 1[ms] to the time-sample?)
                m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());

        swerve.drive(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond, false);

        if(m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()))
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
        m_timer.stop();
        swerve.stop();
    }
}
