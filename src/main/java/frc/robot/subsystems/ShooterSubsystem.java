package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenMotorConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterLeft = new TalonFX(KrakenMotorConstants.kShooterLeftDeviceId);
    private final TalonFX m_shooterRight = new TalonFX(KrakenMotorConstants.kShooterRightDeviceId);

    public ShooterSubsystem() {
        m_shooterLeft.setInverted(true);
        m_shooterLeft.setNeutralMode(NeutralModeValue.Coast);
        m_shooterLeft.setVoltage(12);
        m_shooterLeft.set(0);

        m_shooterRight.setInverted(false);
        m_shooterRight.setNeutralMode(NeutralModeValue.Coast);
        m_shooterRight.setVoltage(12);
        m_shooterRight.set(0);

    }

    public void runShooter(double speed) {
        m_shooterLeft.set(speed);
        m_shooterRight.set(speed);
    }

    public void spinShootNote(double rightSpeed, double leftSpeed)
    {
      m_shooterLeft.set(leftSpeed);
      m_shooterRight.set(rightSpeed);
    }

}