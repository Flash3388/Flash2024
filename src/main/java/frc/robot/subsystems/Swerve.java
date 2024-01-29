package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.jmath.ExtendedMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.SwerveModule;
import org.opencv.core.Mat;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class Swerve extends Subsystem {

    private static final double OFFSET = 0.37;
    public static final double MAX_SPEED = 4.4196;

    private final SwerveModule[] swerveModules; //The array should contain the modules by
    // front-left, front-right, back-left, back-right
    private final WPI_Pigeon2 gyro;


    private final SwerveDriveKinematics swerveDriveKinematics;
    private double currentAngle;
    private PidController pid;

    private SwerveDriveOdometry odometer;
    private Limelight limelight = new Limelight(this);


    public Swerve(SwerveModule[] swerveModules, WPI_Pigeon2 gyro) {
        this.swerveModules = swerveModules;
        this.gyro = gyro;
        pid = new PidController(RunningRobot.getControl().getClock(), 0.01, 0, 0, 0);

        Translation2d fL = new Translation2d(OFFSET, OFFSET);
        Translation2d fR = new Translation2d(OFFSET, -OFFSET);
        Translation2d bL = new Translation2d(-OFFSET, OFFSET);
        Translation2d bR = new Translation2d(-OFFSET, -OFFSET);

        swerveDriveKinematics = new SwerveDriveKinematics(fL, fR, bL, bR);

        this.gyro.reset();
        currentAngle = gyro.getAngle();
        //resetWheels();

        odometer = new SwerveDriveOdometry(
                swerveDriveKinematics,
                new Rotation2d(0),
                getModulePositions(), new Pose2d(0,0,new Rotation2d(Math.toRadians(0))));
    }

    public double getHeadingDegrees() {
        return gyro.getAngle();
    }

    public void setHeadingDegrees() {
        this.gyro.setYaw(0);
    }

    public void resetDistancePassed() {
        swerveModules[0].resetDistancePassed();
    }

    public double getDistancePassedMeters() {
        return -swerveModules[0].getDistancePassedMeters();
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            this.swerveModules[i].stop();
        }
    }

    public void move(double drive, double rotation) {
        for (int i = 0; i < 4; i++) {
            this.swerveModules[i].move(drive, rotation);
        }
    }

    public void resetAllEncoders() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].resetEncoders();
        }
    }

    public void setDesiredStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            this.swerveModules[i].setDesiredState(states[i]);
        }
    }

    public void drive(double speedY, double speedX, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            if (rotation == 0) {
                if (!ExtendedMath.constrained(getHeadingDegrees(), currentAngle - 2, currentAngle + 2)) {
                    rotation = -ExtendedMath.constrain(pid.applyAsDouble(getHeadingDegrees(), currentAngle), -0.1, 0.1);
                }
            } else {
                currentAngle = getHeadingDegrees();
            }


            swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(speedY, speedX, rotation, Rotation2d.fromDegrees(-getHeadingDegrees())));

        } else {
            swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(speedY, speedX, rotation));
        }
        setDesiredStates(swerveModuleStates);

    }


    public void drive(double speedY, double speedX, double rotation) {
        SwerveModuleState[] swerveModuleStates;
        swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(speedY, speedX, rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
        currentAngle = getHeadingDegrees();

        setDesiredStates(swerveModuleStates);
    }

    public void resetWheels() {
        SwerveModuleState[] startingPosition = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        };
        setDesiredStates(startingPosition);
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDriveKinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getModulePosition();
        }
        return positions;
    }

    public void updateOdometer() {
        odometer.update(
                getSwerveRotation2D(),
                getModulePositions());
    }
    public void setOdometer(Rotation2d gyro, Pose2d pose2d) {
        odometer.resetPosition(gyro, getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }
    public void resetOdometer() {
        odometer.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    public Rotation2d getSwerveRotation2D() {
        return new Rotation2d(
                Math.toRadians(this.getHeadingDegrees()));
    }

    public Pose2d getPose2D() {
        return new Pose2d(
                odometer.getPoseMeters().getTranslation(),
                Rotation2d.fromDegrees(0));
    }

    public void print() {
        SmartDashboard.putNumber("FL Heading", swerveModules[0].getHeadingDegrees());
        SmartDashboard.putNumber("FR Heading", swerveModules[1].getHeadingDegrees());
        SmartDashboard.putNumber("RL Heading", swerveModules[2].getHeadingDegrees());
        SmartDashboard.putNumber("RR Heading", swerveModules[3].getHeadingDegrees());


        SmartDashboard.putNumber("FL Velocity", swerveModules[0].getVelocityRpm());
        SmartDashboard.putNumber("FR Velocity", swerveModules[1].getVelocityRpm());
        SmartDashboard.putNumber("RL Velocity", swerveModules[2].getVelocityRpm());
        SmartDashboard.putNumber("RR Velocity", swerveModules[3].getVelocityRpm());
        SmartDashboard.putNumber("FL abs", swerveModules[0].getAbsEncoder());
        SmartDashboard.putNumber("FR abs", swerveModules[1].getAbsEncoder());
        SmartDashboard.putNumber("RL abs", swerveModules[2].getAbsEncoder());
        SmartDashboard.putNumber("RR abs", swerveModules[3].getAbsEncoder());

        SmartDashboard.putNumber("Gyro", getHeadingDegrees());
        SmartDashboard.putNumber("currentAng", currentAngle);
    }







}


