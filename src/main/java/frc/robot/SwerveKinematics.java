package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.*;

/**
 * <p>Class to perform forward kinematics based on given chassis speeds (x, y, and angular velocity).</p>
 * <strong>All swerve module states will be returned in the order: front left, front right, back left, back right</strong>
 * @see SwerveModule
 */
public class SwerveKinematics {
    
    private final SwerveDriveKinematics m_kinematics;

    /**
     * Initializes object based off of track width and wheel base saved in {@link Constants}
     */
    public SwerveKinematics() {
        Translation2d frontLeft = new Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2);
        Translation2d frontRight = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2);
        Translation2d backLeft = new Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2);
        Translation2d backRight = new Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2);

        m_kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Get swerve module states through forward kinematics based on desired chassis speeds.
     * @param chassisSpeeds The expected velocities of the chassis (x, y, angular).
     * @return An array of the swerve module states normalized based on the max possible velocity.
     */
    public SwerveModuleState[] getSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states =  m_kinematics.toSwerveModuleStates(chassisSpeeds);

        // Swerve drive specialties says modules can reach 12 ft/s (3.6576 m/s)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);

        return states;
    }
}
