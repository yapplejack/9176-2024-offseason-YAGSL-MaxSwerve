package swervelib.imu;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import swervelib.imu.SwerveIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;


public class IMUVelocity {
    private final AHRS gyro;
    private final LinearFilter velocityFilter;
    private final Notifier  notifier;

    private boolean firstCycle = true;
    private double timestamp = 0.0;
    private Rotation2d position = new Rotation2d();
    private double velocity = 0.0;
    private double updateCount = 0.0;

    public IMUVelocity(SwerveIMU gyro)
    {
        this(gyro, 1.0/60.0, 5);
    }

    public IMUVelocity(SwerveIMU gyro, double periodSeconds, int averagingTaps)
    {
        if (gyro.getIMU() == AHRS.class)
        {
            this.gyro = AHRS.class.cast(gyro.getIMU());
        }
        else
        {
            this.gyro = null;
            DriverStation.reportWarning(
            "WARNING: The gyro was not a NAVX",
            false);
        }
        velocityFilter = LinearFilter.movingAverage(averagingTaps);
        notifier = new Notifier(this::update);
        notifier.startPeriodic(periodSeconds);
        timestamp = RobotController.getFPGATime();
    }

    private void update() 
    {
        //check isFresh
        boolean isFresh = updateCount < gyro.getUpdateCount();
        updateCount = gyro.getUpdateCount();
        double newTimestamp = RobotController.getFPGATime();
        Rotation2d newPosition = Rotation2d.fromRadians(gyro.getRotation3d().getZ());

        if (isFresh) {
        synchronized (this) {
                if (!firstCycle) {
                    velocity = velocityFilter.calculate(
                    (newPosition.minus(position).getRadians()) / (newTimestamp - timestamp));
                }
                firstCycle = false;
                timestamp = newTimestamp;
                position = newPosition;
            }
        }
    }

    public synchronized double getVelocity() {
        return velocity;
    }
}
