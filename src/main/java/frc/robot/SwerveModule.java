package frc.robot;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private int m_MotorTransID;
    private int m_MotorRotID;
    private int m_UniversalEncoderID;
    private final CANSparkMax transMotor;
    private final CANSparkMax rotMotor;
    private final RelativeEncoder transEncoder;
    private final RelativeEncoder rotEncoder;
    private final AnalogInput universalEncoder;
    public PIDController rotPID;
    private double universalEncoderOffset;
    private Boolean m_transInverted;
    private Boolean m_rotInverted;
    private Boolean encoderInverted;

    public SwerveModule(int motorTransID, int motorRotID, int universalEncoderID,
     Boolean transInverted, Boolean rotInverted, double universalEncoderOffsetinit,
     Boolean universalEncoderInverted){
        
        resetEncoders();

        this.encoderInverted = universalEncoderInverted;
        this.m_MotorTransID = motorTransID;
        this.m_UniversalEncoderID = universalEncoderID;
        this.m_MotorRotID = motorRotID;
        this.m_transInverted = transInverted;
        this.m_rotInverted = rotInverted;
        this.universalEncoderOffset = universalEncoderOffsetinit;


        transMotor = new CANSparkMax(this.m_MotorTransID, MotorType.kBrushless);
        rotMotor = new CANSparkMax(this.m_MotorRotID, MotorType.kBrushless);
        universalEncoder = new AnalogInput(this.m_UniversalEncoderID);

        rotPID = new PIDController(Constants.kRotP,0,0);
        rotPID.enableContinuousInput(-Math.PI,Math.PI) ;

        transMotor.setInverted(this.m_transInverted);
        rotMotor.setInverted(this.m_rotInverted);

        transEncoder = transMotor.getEncoder();
        rotEncoder = rotMotor.getEncoder();

        rotEncoder.setPositionConversionFactor(Constants.angleEncoderConversionFactor);
        rotEncoder.setVelocityConversionFactor(Constants.maxSpeed);
        transEncoder.setPositionConversionFactor(Constants.driveEncoderConversionFactor);
        transEncoder.setVelocityConversionFactor(Constants.maxSpeed);
    }
    public double getTransPosition(){
        return transEncoder.getPosition();
    }
    public double getRotPosition(){
        return rotEncoder.getPosition();
    }
    public double getTransVelocity(){
        return transEncoder.getVelocity();
    }
    public double getRotVelocity(){
        return transEncoder.getVelocity();
    }
    public double getUniversalEncoderRad(){
        double angle = universalEncoder.getVoltage()/RobotController.getVoltage5V();
        angle*=2.0 * Math.PI;
        angle-= universalEncoderOffset;
        if (this.encoderInverted){
            return angle*-1;
        }
        else{
            return angle;
        }
    }
    public void resetEncoders(){
        transEncoder.setPosition(0);
        rotEncoder.setPosition(getUniversalEncoderRad());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getTransVelocity(),new Rotation2d(getRotPosition()));
    }
    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        transMotor.set(desiredState.speedMetersPerSecond/Constants.maxSpeed);
        rotMotor.set(rotPID.calculate(getRotPosition(),desiredState.angle.getRadians()));
    }

}