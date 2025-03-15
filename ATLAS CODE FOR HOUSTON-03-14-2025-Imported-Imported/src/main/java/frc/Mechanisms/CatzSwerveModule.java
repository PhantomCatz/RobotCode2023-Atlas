package frc.Mechanisms;


import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CatzSwerveModule
{
    private final SparkMax STEER_MOTOR;
    private final TalonFX DRIVE_MOTOR;

    private final int MOTOR_ID;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    private PIDController pid;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double currentAngle = 0.0;
    private double angleError = 0.0;
    private double flippedAngleError = 0.0;

    private double command;
    public boolean driveDirectionFlipped = false;

    private double WHEEL_OFFSET;

    public static final SendableChooser<Boolean> chosenState = new SendableChooser<>();

    //current limiting
    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int     STEER_CURRENT_LIMIT_AMPS      = 30;

    private SparkMaxConfig globalConfig = new SparkMaxConfig();
    private final TalonFXConfiguration config = new TalonFXConfiguration();


    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset)
    {
        STEER_MOTOR = new SparkMax(steerMotorID, MotorType.kBrushless);
        DRIVE_MOTOR = new TalonFX(driveMotorID);

        globalConfig.smartCurrentLimit(STEER_CURRENT_LIMIT_AMPS);
        globalConfig.idleMode(IdleMode.kCoast);
        globalConfig.voltageCompensation(12);
        updateConfig();

        
        config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Set current limit
        swerveModuleCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        
        MagEncPWMInput = new DigitalInput(encoderDIOChannel);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        pid = new PIDController(kP, kI, kD);

        WHEEL_OFFSET = offset;

        //for shuffleboard
        MOTOR_ID = steerMotorID;

        DRIVE_MOTOR.getConfigurator().apply(config, 1.0);
        
    }

    private void updateConfig(){
        STEER_MOTOR.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

    public void resetMagEnc()
    {
        System.out.println("magEnc reset called but currently unavailable!!!!!!!!!");
        // magEnc.reset();
    }

    public void setBrakeMode()
    {
        globalConfig.idleMode(IdleMode.kBrake);
        //DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake); //REMOVE AFTER TESTING

    }

    public void setOffset(double offset)
    {
        WHEEL_OFFSET = offset;
    }

    public void setCoastMode()
    {
        globalConfig.idleMode(IdleMode.kCoast);
        //DRIVE_MOTOR.setNeutralMode(NeutralMode.Coast); //REMOVE AFTER TESTING
    }

    public double closestAngle(double startAngle, double targetAngle)
    {
        // get direction
        double error = targetAngle % 360.0 - startAngle % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(error) > 180.0)
        {
            error = -(Math.signum(error) * 360.0) + error;
            //closest angle shouldn't be more than 180 degrees. If it is, use other direction
            if(error > 180.0)
            {
                error -= 360;
            }
        }

        return error;
    }

    public void setWheelAngle(double target, double gyroAngle)
    {
        currentAngle = ((magEnc.get() - WHEEL_OFFSET) * 360.0) - gyroAngle;
        // find closest angle to target angle
        angleError = closestAngle(currentAngle, target);

        // find closest angle to target angle + 180
        flippedAngleError = closestAngle(currentAngle, target + 180.0);

        // if the closest angle to target is shorter
        if (Math.abs(angleError) <= Math.abs(flippedAngleError))
        {
            driveDirectionFlipped = false;
            command = pid.calculate(currentAngle, currentAngle + angleError);
        }
        // if the closest angle to target + 180 is shorter
        else
        {
            driveDirectionFlipped = true;
            command = pid.calculate(currentAngle, currentAngle + flippedAngleError);
        }

        command = -command / (180 * kP); //scale down command to a range of -1 to 1
        STEER_MOTOR.set(command);
    }

    public void setSteerPower(double pwr)
    {
        STEER_MOTOR.set(pwr);
    }

    public void setDrivePower(double pwr)
    {
        if(driveDirectionFlipped == true)
        {
            pwr = -pwr;
        }
        DRIVE_MOTOR.set(-pwr);
    }
    
    public double getEncValue()
    {
        return magEnc.get();
    }

    public double getDrvDistanceRaw()
    {
        return DRIVE_MOTOR.getPosition().getValueAsDouble();//getSelectedSensorPosition();
    }

    public double getDrvDistance()
    {
        if(driveDirectionFlipped)
        {
            return DRIVE_MOTOR.getPosition().getValueAsDouble();
        }
        else
        {
            return -DRIVE_MOTOR.getPosition().getValueAsDouble();
        }
    }

    public void resetDrvDistance()
    {
        int i = 0;

        DRIVE_MOTOR.setPosition(0.0);//setSelectedSensorPosition(0.0);
        while(Math.acos(DRIVE_MOTOR.getPosition().getValueAsDouble()) > 1.0)
        {
            i++;
            if(i >= 3000)
            {
                resetDrvDistance();
            }
        }
    }

    public double getDrvVelocity()
    {
        return DRIVE_MOTOR.getVelocity().getValueAsDouble();
    }
    
    public double getAngle()
    {
        return magEnc.get();//currentAngle
    }

    public double getWheelAngle()
    {
        return (magEnc.get() - WHEEL_OFFSET) * 360.0;
    }

    public double getStrPwr()
    {
        return STEER_MOTOR.getAppliedOutput();
    }

    public double getError()
    {
        return angleError;
    }

    public double getFlipError()
    {
        return flippedAngleError;
    }

    public void smartDashboardModules()
    {
        SmartDashboard.putNumber(MOTOR_ID + " Wheel Angle", (currentAngle));
    }

    public void smartDashboardModules_DEBUG()
    {
        SmartDashboard.putNumber(MOTOR_ID + " Mag Encoder", magEnc.get() );
        //SmartDashboard.putBoolean(motorID + " Flipped", driveDirectionFlipped);
    }

    /*Auto Balance */
    public void reverseDrive(Boolean reverse)
    {
        DRIVE_MOTOR.setInverted(reverse);
    }
}
