package frc.robot.actions;
import frc.robot.subSystems.Swerve;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;


/**
 * base action following a path.
 */
public class FollowPathAction extends ActionBase {
    private final Timer timer = new Timer();
    private final PathPlannerPath path;
    private final PathFollowingController controller;
    private final ReplanningConfig replanningConfig;
    private PathPlannerTrajectory generatedTrajectory;

    private Swerve swerve;
    /**
     * Construct a base path following command
     *
     * @param path             The path to follow
     * @param controller       Path following controller that will be used to follow the path
     * @param replanningConfig Path replanning configuration
     * @param swerve           the Subsystems required by this command, the swerve drive system.
     */
    public FollowPathAction(
            PathPlannerPath path,
            PathFollowingController controller,
            ReplanningConfig replanningConfig,
            Swerve swerve){
        this.swerve = swerve;
        this.path = path;
        this.controller = controller;
        this.replanningConfig = replanningConfig;
        requires(swerve);
    }
    @Override
    public void initialize(ActionControl control) {

        Pose2d currentPose = swerve.getPose();
        ChassisSpeeds currentSpeeds = swerve.getSpeeds();

        controller.reset(currentPose, currentSpeeds);

        ChassisSpeeds fieldSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
        Rotation2d currentHeading =
                new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Rotation2d targetHeading =
                path.getPoint(1).position.minus(path.getPoint(0).position).getAngle();
        Rotation2d headingError = currentHeading.minus(targetHeading);
        boolean onHeading =
                Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.25
                        || Math.abs(headingError.getDegrees()) < 30;

        if (!path.isChoreoPath()
                && replanningConfig.enableInitialReplanning
                && !(currentPose.getTranslation().getDistance(path.getPoint(0).position) < 0.25
                && onHeading)) {
            replanPath(currentPose, currentSpeeds);
        } else {
            generatedTrajectory = path.getTrajectory(currentSpeeds, currentPose.getRotation());
            PathPlannerLogging.logActivePath(path);
            PPLibTelemetry.setCurrentPath(path);
        }

        timer.reset();
        timer.start();
    }

    @Override
    public void execute(ActionControl control) {

        double currentTime = timer.get();
        PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
        if (!controller.isHolonomic() && path.isReversed()) {
            targetState = targetState.reverse();
        }

        Pose2d currentPose = swerve.getPose();
        ChassisSpeeds currentSpeeds = swerve.getSpeeds();

        if (!path.isChoreoPath() && replanningConfig.enableDynamicReplanning) {
            double previousError = Math.abs(controller.getPositionalError());
            double currentError = currentPose.getTranslation().getDistance(targetState.positionMeters);

            if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
                    || currentError - previousError
                    >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
                replanPath(currentPose, currentSpeeds);
                timer.reset();
                targetState = generatedTrajectory.sample(0);
            }
        }

        ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

        double currentVel =
                Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        PPLibTelemetry.setCurrentPose(currentPose);
        PathPlannerLogging.logCurrentPose(currentPose);

        if (controller.isHolonomic()) {
            PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose());
            PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());
        } else {
            PPLibTelemetry.setTargetPose(targetState.getDifferentialPose());
            PathPlannerLogging.logTargetPose(targetState.getDifferentialPose());
        }

        PPLibTelemetry.setVelocities(
                currentVel,
                targetState.velocityMps,
                currentSpeeds.omegaRadiansPerSecond,
                targetSpeeds.omegaRadiansPerSecond);
        PPLibTelemetry.setPathInaccuracy(controller.getPositionalError());

        swerve.pathDrive(targetSpeeds);
        if(timer.hasElapsed(generatedTrajectory.getTotalTimeSeconds())){
            control.finish();
        }
    }

    @Override
    public void end(FinishReason reason) {
        timer.stop();

        // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
        // the command to smoothly transition into some auto-alignment routine
        if (!reason.isInterrupt() && path.getGoalEndState().getVelocity() < 0.1) {
            swerve.stop();
        }

        PathPlannerLogging.logActivePath(null);

    }

    private void replanPath(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        PathPlannerPath replanned = path.replan(currentPose, currentSpeeds);
        generatedTrajectory =
                new PathPlannerTrajectory(replanned, currentSpeeds, currentPose.getRotation());
        PathPlannerLogging.logActivePath(replanned);
        PPLibTelemetry.setCurrentPath(replanned);
    }
}
