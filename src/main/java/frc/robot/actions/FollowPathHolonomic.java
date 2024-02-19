package frc.robot.actions;

import frc.robot.subSystems.Swerve;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.subSystems.Swerve;

public class FollowPathHolonomic extends FollowPathAction {
    /**
     * Construct a path following action that will use a holonomic drive controller for holonomic
     * drive trains
     *
     * @param path The path to follow
     * @param translationConstants PID constants for the translation PID controllers
     * @param rotationConstants PID constants for the rotation controller
     * @param maxModuleSpeed The max speed of a drive module in meters/sec
     * @param driveBaseRadius The radius of the drive base in meters. For swerve drive, this is the
     *     distance from the center of the robot to the furthest module. For mecanum, this is the
     *     drive base width / 2
     * @param period Period of the control loop in seconds, default is 0.02s
     * @param replanningConfig Path replanning configuration
     * @param swerve Subsystems required by this action, the swerve driving system.
     */
    public FollowPathHolonomic(
            PathPlannerPath path,
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            double maxModuleSpeed,
            double driveBaseRadius,
            double period,
            ReplanningConfig replanningConfig,
            Swerve swerve) {
        super(
                path,
                new PPHolonomicDriveController(
                        translationConstants, rotationConstants, period, maxModuleSpeed, driveBaseRadius),
                replanningConfig,
                swerve);
    }

    /**
     * Construct a path following action that will use a holonomic drive controller for holonomic
     * drive trains
     *
     * @param path The path to follow
     * @param translationConstants PID constants for the translation PID controllers
     * @param rotationConstants PID constants for the rotation controller
     * @param maxModuleSpeed The max speed of a drive module in meters/sec
     * @param driveBaseRadius The radius of the drive base in meters. For swerve drive, this is the
     *     distance from the center of the robot to the furthest module. For mecanum, this is the
     *     drive base width / 2
     * @param replanningConfig Path replanning configuration
     * @param swerve Subsystem required by this action, the swerve subsytem.
     */
    public FollowPathHolonomic(
            PathPlannerPath path,
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            double maxModuleSpeed,
            double driveBaseRadius,
            ReplanningConfig replanningConfig,
            Swerve swerve) {
        this(
                path,
                translationConstants,
                rotationConstants,
                maxModuleSpeed,
                driveBaseRadius,
                0.02,
                replanningConfig,
                swerve);
    }
    /**
     * Construct a path following action that will use a holonomic drive controller for holonomic
     * drive trains
     *
     * @param path The path to follow
     * @param config Holonomic path follower configuration
     * @param swerve Subsystems required by this action, the swerve subsystem
     */
    public FollowPathHolonomic(
            PathPlannerPath path,
            HolonomicPathFollowerConfig config,
            Swerve swerve) {
        this(
                path,
                config.translationConstants,
                config.rotationConstants,
                config.maxModuleSpeed,
                config.driveBaseRadius,
                config.period,
                config.replanningConfig,
                swerve);
    }



}
