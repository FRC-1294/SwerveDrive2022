package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
    private CANSparkMax angle;
    private CANSparkMax drive;
    private CANPIDController anglePID;
    private CANPIDController drivePID;
    private final Gains defaultPID;
    
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("dataTable");
    private final NetworkTableEntry angleEntry;
    private final NetworkTableEntry driveEntry;

    //init sparks
    public SwerveModule(int steerCAN, int driveCAN) {
        //Sparks
        angle = new CANSparkMax(steerCAN, MotorType.kBrushless);
        drive = new CANSparkMax(driveCAN, MotorType.kBrushless);

        //PID
        anglePID = angle.getPIDController();
        drivePID = drive.getPIDController();

        defaultPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -0.5, 0.5, 0);
        setPidControllers(drivePID, defaultPID, defaultPID.kSlot);
        setPidControllers(anglePID, defaultPID, defaultPID.kSlot);

        //Settings
        angle.restoreFactoryDefaults(true);
        drive.restoreFactoryDefaults(true);

        angle.getEncoder();
        angle.getEncoder().setPositionConversionFactor(Constants.angleEncoderConversionFactor);
        drive.getEncoder();

        angle.setSmartCurrentLimit(60);
        drive.setSmartCurrentLimit(60);

        angle.setOpenLoopRampRate(1);
        drive.setOpenLoopRampRate(1);

        //data entries
        angleEntry = networkTable.getEntry("Angle:" + steerCAN);
        driveEntry = networkTable.getEntry("Drive:" + driveCAN);
    }

    //getting the current state of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getEncoder().getVelocity(), new Rotation2d(angle.getEncoder().getPosition()));
    }

    //for setting the swerve module to the wanted statee
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(angle.getEncoder().getPosition()));

        drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        anglePID.setReference(state.angle.getDegrees(), ControlType.kPosition);

        angleEntry.setNumber(angle.get());
        driveEntry.setNumber(drive.get());
    }

    //initallizing pid controllers
    private void setPidControllers (CANPIDController pidController, Gains pidSet, int slot) {
        pidController.setP(pidSet.kP, slot);
        pidController.setI(pidSet.kI, slot);
        pidController.setD(pidSet.kD, slot);
        pidController.setIZone(pidSet.kIz, slot);
        pidController.setFF(pidSet.kFF, slot);
        pidController.setOutputRange(pidSet.kMinOutput, pidSet.kMaxOutput, slot);
    }
}