package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveModule {
    private CANSparkMax angle;
    private CANSparkMax drive;
    private CANPIDController anglePID;
    private CANPIDController drivePID;
    private final Gains defaultPID;

    private double[] loc;
    private double setVelocity;
    private double setAngle;
    
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("dataTable");
    //init sparks
    public SwerveModule(int steerCAN, int driveCAN, double[] loc) {
        //Sparks
        angle = new CANSparkMax(steerCAN, MotorType.kBrushless);
        drive = new CANSparkMax(driveCAN, MotorType.kBrushless);

        //PID
        anglePID = angle.getPIDController();
        drivePID = drive.getPIDController();

        defaultPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -0.5, 0.5, 0);
        setPidControllers(drivePID, defaultPID, defaultPID.kSlot);
        setPidControllers(anglePID, defaultPID, defaultPID.kSlot);

        //location
        this.loc = loc;

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
    }

    public double getSetVelocity() {
        return this.setVelocity;
    } 
    
    public double getSetAngle() {
        return this.setAngle;
    }

    //calculate velocity and angle, then set state of module
    public void setModuleState(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        //deadzone filtering
        if (Math.abs(xSpeed) < Constants.deadzone) xSpeed = 0;
        if (Math.abs(ySpeed) < Constants.deadzone) ySpeed = 0;
        if (Math.abs(rot) < Constants.deadzone) rot = 0;

        //normalize velocity to range of -1 to 1
        double factor = Math.sqrt(Math.pow(Math.abs(xSpeed)+Math.abs(rot), 2) + Math.pow(Math.abs(ySpeed)+Math.abs(rot), 2));

        if (factor > 1) {
            factor = 1/factor;

            xSpeed *= factor;
            ySpeed *= factor;
            rot *= factor;
        }

        //CALCULATE SUM OF VECTORS
        double totalXSpeed = 0;
        double totalYSpeed = 0;
        //X&Y:
        totalXSpeed += xSpeed;
        totalYSpeed += ySpeed;
        //ROT vectors:
        //calculate angle through tanget, add or subtract 90 based on front/back loc and rotation direction
        double rotAngle = Math.toDegrees(Math.atan(loc[0]/loc[1])) + 90*getSign(loc[1])*getSign(rot);
        //calculate velocity magnitude through trig (RELATIVE TO Y-AXIS)
        totalXSpeed += Math.sin(Math.toRadians(rotAngle)) * Math.abs(rot)*Math.sqrt(2);
        totalYSpeed += Math.cos(Math.toRadians(rotAngle)) * Math.abs(rot)*Math.sqrt(2);

        //calculate FINAL velocity and angle
        this.setVelocity = Math.sqrt(Math.pow(totalXSpeed, 2) + Math.pow(totalYSpeed, 2)) * Constants.maxSpeed;
        this.setAngle = Math.toDegrees(Math.atan(totalXSpeed/totalYSpeed));

        //handle divison by zero
        if (Double.isNaN(this.setAngle)) this.setAngle = 0;
        //handle lower quadrants (> 90 degrees)
        if (totalYSpeed < 0) this.setAngle = (180-Math.abs(this.setAngle)) * getSign(totalXSpeed);

        applyState();
    }

    //returns +1 or -1 based on num's sign
    private double getSign(double num) {
        double sign = num/Math.abs(num);
        if (Double.isNaN(sign)) sign = 1;

        return sign;
    }

    //for setting the swerve module to the wanted state
    public void applyState() {
        // Optimize the reference state to avoid spinning further than 90 degrees
        // SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(angle.getEncoder().getPosition()));

        drivePID.setReference(this.setVelocity, ControlType.kVelocity);
        anglePID.setReference(this.setAngle, ControlType.kPosition);
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