package frc.robot.subSystems;

import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;

import java.util.ArrayList;

public class TrajectoryExample {

    private SwerveDriveKinematics swerveDriveKinematics;

    public TrajectoryExample(SwerveDriveKinematics swerveDriveKinematics) {
        this.swerveDriveKinematics = swerveDriveKinematics;
    }

    public Trajectory createSimpleTrajectory(){
        var start = new Pose2d(0,0, // start point
                Rotation2d.fromDegrees(0));

        var end = new Pose2d(4, 0, // end point
                Rotation2d.fromDegrees(0));


        SwerveDriveKinematicsConstraint d = new SwerveDriveKinematicsConstraint(swerveDriveKinematics, 4.19);

        TrajectoryConfig config = new TrajectoryConfig(4, 2); // config
        config.addConstraint(d);
        config.setKinematics(swerveDriveKinematics);


        var trajectory = TrajectoryGenerator.generateTrajectory( // the trajectory
                start,
                new ArrayList<Translation2d>(),
                end,
                config);

        return trajectory;
    }

    public HolonomicDriveController getHolonomicController(){
        var controller = new HolonomicDriveController(
                new PIDController(1, 0, 0), new PIDController(1, 0, 0),
                new ProfiledPIDController(1, 0, 0,
                        new TrapezoidProfile.Constraints(6.28, 3.14)));

        return controller;
    }
}
