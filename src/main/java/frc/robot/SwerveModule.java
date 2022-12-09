package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.*;

public class SwerveModule {
    
    private final CANSparkMax m_drive, m_steer;

    /**
     * Create a new swerve module from the ID's of its controlling Spark MAXs
     * @param driveId ID of the Spark MAX controlling the driving motor
     * @param steerId ID of the Spark MAX controlling the motor steering the module
     */
    public SwerveModule(int driveId, int steerId) {
        m_drive = new CANSparkMax(driveId, MotorType.kBrushless);
        m_steer = new CANSparkMax(steerId, MotorType.kBrushless);
        
        // Set velocity conversion factor to change encoder units from RPM to m/s.
        m_drive.getEncoder().setVelocityConversionFactor(60 * Math.PI * WHEEL_DIAMETER / 100);

        RelativeEncoder canCoder = m_steer.getAlternateEncoder(CANCODER_COUNTS);

        // Set position conversion factor to change encoder units from counts to degrees.
        canCoder.setPositionConversionFactor(1 / 360f);

        m_steer.getPIDController().setFeedbackDevice(canCoder);
    }

    /**
     * Sets a swerve module to a desired state.
     * @param state The desired state (should be provided by {@link SwerveKinematics} class)
     */
    public void setState(SwerveModuleState state) {
        m_drive.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        m_steer.getPIDController().setReference(state.angle.getDegrees(), ControlType.kPosition);
    }
}
