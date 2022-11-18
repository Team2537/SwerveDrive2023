package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_frontLeftDrive, m_frontRightDrive, m_backLeftDrive, m_backRightDrive;
    private final CANSparkMax m_frontLeftPivot, m_frontRightPivot, m_backLeftPivot, m_backRightPivot;

    public SwerveSubsystem() {
        MotorType type = MotorType.kBrushless;

        m_frontLeftDrive = new CANSparkMax(FRONT_LEFT_DRIVE, type);
        m_frontRightDrive = new CANSparkMax(FRONT_RIGHT_DRIVE, type);
        m_backLeftDrive = new CANSparkMax(BACK_LEFT_DRIVE, type);
        m_backRightDrive = new CANSparkMax(BACK_RIGHT_DRIVE, type);

        m_frontLeftPivot = new CANSparkMax(FRONT_LEFT_PIVOT, type);
        m_frontRightPivot = new CANSparkMax(FRONT_RIGHT_PIVOT, type);
        m_backLeftPivot = new CANSparkMax(BACK_LEFT_PIVOT, type);
        m_backRightPivot = new CANSparkMax(BACK_RIGHT_PIVOT, type);

        

        m_frontLeftDrive.burnFlash();
        m_frontRightDrive.burnFlash();
        m_backLeftDrive.burnFlash();
        m_backRightDrive.burnFlash();

        m_frontLeftPivot.burnFlash();
        m_frontRightPivot.burnFlash();
        m_backLeftPivot.burnFlash();
        m_backRightPivot.burnFlash();
    }

    public void drive(double forward, double strafe, double rotation) {
        double a = strafe - rotation * WHEEL_BASE / 2;
        double b = strafe + rotation * WHEEL_BASE / 2;
        double c = forward - rotation * TRACK_WIDTH / 2;
        double d = forward + rotation * TRACK_WIDTH / 2;

        double speedFL = Math.sqrt(b*b + d*d);
        double speedFR = Math.sqrt(b*b + c*c);
        double speedBL = Math.sqrt(a*a + d*d);
        double speedBR = Math.sqrt(a*a + c*c);

        if (speedFL + speedFR + speedBL + speedBR > 4) {
            double greatest = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));

            speedFL /= greatest;
            speedFR /= greatest;
            speedBL /= greatest;
            speedBR /= greatest;
        }

        setDriveMotors(speedFL, speedFR, speedBL, speedBR);

        double angleFL = Math.toDegrees(Math.atan2(b, d));
        double angleFR = Math.toDegrees(Math.atan2(b, c));
        double angleBL = Math.toDegrees(Math.atan2(a, d));
        double angleBR = Math.toDegrees(Math.atan2(a, c));

        pivotMotors(angleFL, angleFR, angleBL, angleBR);
    }

    private void setDriveMotors(double fl, double fr, double bl, double br) {
        m_frontLeftDrive.set(fl);
        m_frontRightDrive.set(fr);
        m_backLeftDrive.set(bl);
        m_backRightDrive.set(br);
    }

    private void pivotMotors(double fl, double fr, double bl, double br) {
        m_frontLeftPivot.getPIDController().setReference(fl, ControlType.kPosition);
        m_frontRightPivot.getPIDController().setReference(fr, ControlType.kPosition);
        m_backLeftPivot.getPIDController().setReference(bl, ControlType.kPosition);
        m_backRightPivot.getPIDController().setReference(br, ControlType.kPosition);
    }

    private void tunePID() {
        // TODO: Tune through Spark Max GUI then hard code the values here.
    }
}
