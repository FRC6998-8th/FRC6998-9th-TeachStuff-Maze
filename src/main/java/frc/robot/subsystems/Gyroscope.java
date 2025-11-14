package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {
  private final AHRS gyro;
  private AHRS microIMU;

  public Gyroscope() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    // microIMU = new AHRS(NavXComType.kUSB1);

    // microIMU.close();

    //gyro.reset();
    // microIMU.reset();
  }

  public double GetDriveRoll() {
    return gyro.getRoll();
  }

  public double GetDrivePitch() {
    return gyro.getPitch();
  }

  public double GetMazePitch() {    
    return microIMU.getPitch();
  }

  public double GetMazeRoll() {
    return microIMU.getRoll();
  }

  public double GetMazePitchVelocity() {
    return microIMU.getRawGyroX();
  }

  public double GetMazeRollVelocity() {
    return microIMU.getRawGyroY();
  }

  public Command ResetDriveGyro() {
    return runOnce(() -> {
      gyro.reset();
    });
  }

  public Command ResetMazeGyro() {

    return runOnce(() -> {
      microIMU.reset();
    });
  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("isconnected", gyro.isConnected());
    SmartDashboard.putBoolean("isCalibrating", gyro.isCalibrating());
    SmartDashboard.putNumber("port number", gyro.getPort());
    SmartDashboard.putNumber("yaw", gyro.getYaw());
    SmartDashboard.putNumber("update count", gyro.getUpdateCount());
    SmartDashboard.putNumber("update rate", gyro.getActualUpdateRate());
    SmartDashboard.putNumber("DriveRoll", gyro.getRoll());
    SmartDashboard.putNumber("DrivePitch", gyro.getPitch());
  }

  @Override
  public void simulationPeriodic() {
    // This mellllthod will be called once per scheduler run during simulation

  }
}
