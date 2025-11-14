package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class MazeSubsystem extends SubsystemBase {
    private double outPutPitch = 0;
    private double outPutRoll = 0;

    private final SparkMax motorPitch;
    private final SparkMax motorRoll;

    private final Gyroscope m_Gyroscope;

    private SysIdRoutine sysIdRoutine;
    
    private MutVoltage mutVoltage = Volts.mutable(0);
    private MutAngularVelocity mutAngularVelocity = RotationsPerSecond.mutable(0);
    private MutAngle mutPosition = Rotations.mutable(0);

    public MazeSubsystem(String sysIdStatus) {
        m_Gyroscope = new Gyroscope();

        motorPitch = new SparkMax(Constants.Motors.PITCH_DEVICE_ID,  MotorType.kBrushless);
        motorRoll = new SparkMax(Constants.Motors.ROLL_DEVICE_ID, MotorType.kBrushless);

        motorPitch.configure(Constants.Motors.configPitch, Constants.Motors.kReset, Constants.Motors.kPersist);
        motorRoll.configure(Constants.Motors.configRoll, Constants.Motors.kReset, Constants.Motors.kPersist);

        motorPitch.getEncoder().setPosition(0);
        motorRoll.getEncoder().setPosition(0);

        if (sysIdStatus == "Roll") {
            SetRollSysId();
        } else if (sysIdStatus == "Pitch") {
            SetPitchSysId();
        }
    }

    public void SetPitchSysId() {
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                 null), 
            new SysIdRoutine.Mechanism(
                volts -> {motorPitch.setVoltage(volts);},
                log -> {
                    log.motor("Pitch")
                        .voltage(mutVoltage.mut_replace(
                            motorPitch.getBusVoltage() * motorPitch.getAppliedOutput(), Volts))
                        .angularPosition(mutPosition.mut_replace(
                            m_Gyroscope.GetMazePitch(), Degrees))
                        .angularVelocity(mutAngularVelocity.mut_replace(
                            m_Gyroscope.GetMazePitchVelocity(), DegreesPerSecond));
                }
            , this));
    }

    public void SetRollSysId() {
        sysIdRoutine = new SysIdRoutine( 
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null),
            new SysIdRoutine.Mechanism(
                volts -> {motorRoll.setVoltage(volts);},
                log -> {
                    log.motor("Roll")
                        .voltage(mutVoltage.mut_replace(
                            motorRoll.getBusVoltage() * motorRoll.getAppliedOutput(), Volts))
                        .angularPosition(mutPosition.mut_replace(
                            m_Gyroscope.GetMazeRoll(), Degrees))
                        .angularVelocity(mutAngularVelocity.mut_replace(
                            m_Gyroscope.GetMazeRollVelocity(), DegreesPerSecond));
                },
                this));
    }

    public Command SysIdDynamic(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            sysIdRoutine.dynamic(direction);
        });
    }

    public Command SysIdQuastatic(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            sysIdRoutine.quasistatic(direction);
        });
    }

    public Command ResetDriveGyro() {
        return runOnce(() -> {
            m_Gyroscope.ResetDriveGyro();
        });
    }

    public Command ResetMaze() {
        return runOnce(() -> {
            motorPitch.getEncoder().setPosition(0);
            motorRoll.getEncoder().setPosition(0);
        });
    }

    public void Release() {
        outPutRoll = m_Gyroscope.GetDriveRoll()/360;
        outPutPitch = m_Gyroscope.GetDrivePitch()/360;

        SetPitchPosition(outPutPitch);
        SetRollPosition(outPutRoll);
    }

    public void SetRollPosition(double position) {
        motorRoll.getClosedLoopController()
        .setReference(position, ControlType.kMAXMotionPositionControl);
    }

    public void SetPitchPosition(double position) {
        motorPitch.getClosedLoopController()
        .setReference(position, ControlType.kMAXMotionPositionControl);
    }

    public double GetMazePitch() {
        return motorPitch.getEncoder().getPosition();
    }

    public double GetMazeRoll() {
        return motorRoll.getEncoder().getPosition();
    }

    public double GetPitchVelocity() {
        return motorPitch.getEncoder().getVelocity();
    }

    public double GetRollVelocity() {
        return motorRoll.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("MazePitch", GetMazePitch());
        SmartDashboard.putNumber("MazeRoll", GetMazeRoll());
        SmartDashboard.putNumber("SetPointPitch", outPutPitch);
        SmartDashboard.putNumber("SetPointRoll", outPutRoll);
        SmartDashboard.putNumber("PitchOutVol", motorPitch.getAppliedOutput());
        SmartDashboard.putNumber("PitchCurrent", motorPitch.getOutputCurrent());
    }
}
