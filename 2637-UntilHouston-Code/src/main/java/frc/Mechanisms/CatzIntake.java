package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CatzIntake
{
    private WPI_TalonFX wristMtr;
    private WPI_TalonFX rollersMtr;

    private final double THREAD_PERIOD = 0.02;

    private final double SOFT_LIMIT_FORWARD = 70.0;
    private final double SOFT_LIMIT_REVERSE = -5850.0;

    private final double SEMI_MANUAL_RATE = 10.0;

    private final int WRIST_MC_ID   = 31;
    private final int ROLLERS_MC_ID = 30;

    private final double WRIST_MAX_PWR = 1.0;
    private final double ROLLERS_PWR   = 0.4;
    
    private final int WRIST_ENC_CAN_ID = 13; 

    private final double WRIST_ENC_OFFSET = -5092;

    private final double ENC_TO_INTAKE_GEAR_RATIO =  18.0/46.0;
    private final double WRIST_DEGREE_PER_CNT = 360.0 / 4096.0 * ENC_TO_INTAKE_GEAR_RATIO;

    private final double STOW_ENC_POS = 5000.0;
    private final double INTAKE_CUBE_ENC_POS = 55.0; // TBD
    private final double INTAKE_CONE_ENC_POS = -805.0; //TBD
    private final double SCORE_CUBE_ENC_POS = 1570.0; // TBD
    private final double SCORE_CONE_HIGH_ENC_POS = -487.0; //TBD
    private final double SCORE_CONE_MID_ENC_POS = -488.0; //TBD
    private final double SCORE_CONE_LOW_ENC_POS = INTAKE_CONE_ENC_POS; //TBD

    private final double STOW_CUTOFF = 4800.0;

    private final int COMMAND_STATE_STOW = 0;

    private final int COMMAND_STATE_PICKUP_CONE = 1;
    private final int COMMAND_STATE_PICKUP_CUBE = 2;

    private final int COMMAND_STATE_SCORE_LOW_CONE = 10;
    private final int COMMAND_STATE_SCORE_LOW_CUBE = 11;

    private final int COMMAND_STATE_SCORE_MID_CONE = 12;
    private final int COMMAND_STATE_SCORE_MID_CUBE = 13;

    private final int COMMAND_STATE_SCORE_HIGH_CONE = 14;
    private final int COMMAND_STATE_SCORE_HIGH_CUBE = 15;

    private final int COMMAND_STATE_DO_NOTHING = 20;

    //current limiting
    private SupplyCurrentLimitConfiguration wristCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private SupplyCurrentLimitConfiguration rollerCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS_ROLLER            = 40;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS_ROLLER    = 40;

    private final double CM_DEGREE_OFFSET = 4.758;

    //private double wristAngle;

    private final double kP = 0.00009; //0.00036
    private final double kI = 0.000040; //0.000000028
    private final double kD = 0.000007; //0.000055
    
    double maxGravityFF = 0.0725; //0.11
    
    private PIDController pid;
    private double targetPositionEnc = STOW_ENC_POS;
    private double targetPower;
    private Boolean pidEnable = false;

    private double mtrPower;
    private CANCoder wristEncoder;

    private double wristManualThreadPower = 0.0;

    public CatzIntake()
    {
        wristEncoder = new CANCoder(WRIST_ENC_CAN_ID);

        wristMtr = new WPI_TalonFX(WRIST_MC_ID);
        wristMtr.setNeutralMode(NeutralMode.Brake);

        wristMtr.configForwardSoftLimitThreshold(SOFT_LIMIT_FORWARD);
        wristMtr.configReverseSoftLimitThreshold(SOFT_LIMIT_REVERSE);

        //wristMtr.configForwardSoftLimitEnable(true);
        //wristMtr.configReverseSoftLimitEnable(true);

        wristCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);
        wristMtr.configSupplyCurrentLimit(wristCurrentLimit);

        rollerCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS_ROLLER, CURRENT_LIMIT_TRIGGER_AMPS_ROLLER, CURRENT_LIMIT_TIMEOUT_SECONDS);


        rollersMtr = new WPI_TalonFX(ROLLERS_MC_ID);
        rollersMtr.configFactoryDefault();
        rollersMtr.setNeutralMode(NeutralMode.Brake);
        rollersMtr.configSupplyCurrentLimit(rollerCurrentLimit);

        pid = new PIDController(kP, kI, kD);

        startIntakeThread();
    }

    public void resetPID(){
        pidEnable = false;
        pid.reset();
    }

    public void enablePID(boolean set){
        pidEnable = set;
    }

    public boolean getPIDEnabled(){
        return pidEnable;
    
    }

    public void startIntakeThread()
    {
        Thread intakeThread = new Thread(() ->
        {
            while(true)
            {
                if(pidEnable){
                    targetPower = pid.calculate(wristMtr.getSelectedSensorPosition() - WRIST_ENC_OFFSET, targetPositionEnc) + calculateGravityFF();
                    if(targetPositionEnc == STOW_ENC_POS && wristMtr.getSelectedSensorPosition() - WRIST_ENC_OFFSET > STOW_CUTOFF){
                        targetPower = 0.0;
                    }
                    wristMtr.set(ControlMode.PercentOutput, targetPower);
                }
                else
                {
                    wristMtr.set(ControlMode.PercentOutput, wristManualThreadPower);
                }

                Timer.delay(THREAD_PERIOD);    
            }   //End of while(true)
        });
        intakeThread.start();
    }


    /*** TBD - DELETE after prcCmdIntake() finalized  
     
    public void cmdProcIntakeTest(double wristPwr, boolean rollersIn, boolean rollersOut, boolean stow, boolean cube, boolean cone, boolean killPID, boolean softLimitOverride)
    {
        if(Math.abs(wristPwr) >= 0.1 && pidEnable == false)
        {
            pidEnable = false;
            wristManualThreadPower = wristPwr *.5;
        }
        else if(pidEnable == false)
        {
            wristManualThreadPower = 0.0;
        }

        if (Math.abs(wristPwr) >= 0.1 && pidEnable == true)
        {
            if(wristPwr > 0){
                targetPositionEnc = Math.min((targetPositionEnc + wristPwr * SEMI_MANUAL_RATE), SOFT_LIMIT_FORWARD);
            }
            else{
                targetPositionEnc = Math.max((targetPositionEnc + wristPwr * SEMI_MANUAL_RATE), SOFT_LIMIT_REVERSE);
            }

        }
        if(rollersIn)
        {
            rollersIn();
        }
        else if(rollersOut)
        {
            rollersOut();
        }
        else
        {
            rollersOff();
        }

        if(stow)
        {
            pidEnable = true;
            targetPositionEnc = STOW_ENC_POS;
        }
        else if(cube)
        {
            pidEnable = true;
            targetPositionEnc = SCORE_CUBE_ENC_POS;
        }
        else if(cone)
        {
            pidEnable = true;
            targetPositionEnc = SCORE_CONE_HIGH_ENC_POS;
        }

        if(killPID){
            pidEnable = false;
        }

        if(Override){
            wristMtr.configForwardSoftLimitEnable(false);
            wristMtr.configReverseSoftLimitEnable(false);
        }
        else{
            wristMtr.configForwardSoftLimitEnable(true);
            wristMtr.configReverseSoftLimitEnable(true);
        }
    }
    ***/

    public void cmdProcIntake(double wristPwr, boolean rollersIn, boolean rollersOut, boolean killPID, boolean softLimitOverride, int state)
    {
        if(Math.abs(wristPwr) >= 0.1)
        {
            if (pidEnable == true)
            {
                if(wristPwr > 0){
                    targetPositionEnc = Math.min((targetPositionEnc + wristPwr * SEMI_MANUAL_RATE), SOFT_LIMIT_FORWARD);
                }
                else{
                    targetPositionEnc = Math.max((targetPositionEnc + wristPwr * SEMI_MANUAL_RATE), SOFT_LIMIT_REVERSE);
                }
            }
            else
            {
                pidEnable = false;
                wristManualThreadPower = wristPwr * WRIST_MAX_PWR;    
            }
        }
        else if(pidEnable == false)
        {
            wristManualThreadPower = 0.0;
        }

        if(rollersIn)           //TBD - put all wrist stuff together?
        {
            //rollersIn();
        }
        else if(rollersOut)
        {
            //rollersOut();
        }
        else
        {
            rollersOff();
        }

        if(state != COMMAND_STATE_DO_NOTHING)
        {
            pid.reset();
            pidEnable = true;
            
            switch(state){
                case COMMAND_STATE_STOW :
                    targetPositionEnc = STOW_ENC_POS;
                    break;
                    
                case COMMAND_STATE_PICKUP_CONE :
                    targetPositionEnc = INTAKE_CONE_ENC_POS;
                    break;
                    
                case COMMAND_STATE_PICKUP_CUBE :
                    targetPositionEnc = INTAKE_CUBE_ENC_POS;
                    break;
                    
                case COMMAND_STATE_SCORE_LOW_CONE :
                    targetPositionEnc = SCORE_CONE_LOW_ENC_POS;
                    break;
    
                case COMMAND_STATE_SCORE_LOW_CUBE :
                    targetPositionEnc = SCORE_CUBE_ENC_POS;
                    break;
                case COMMAND_STATE_SCORE_MID_CUBE :
                    targetPositionEnc = SCORE_CUBE_ENC_POS;
                    break;
                case COMMAND_STATE_SCORE_HIGH_CUBE:
                    targetPositionEnc = SCORE_CUBE_ENC_POS;
                    break;
    
                case COMMAND_STATE_SCORE_MID_CONE :
                    targetPositionEnc = SCORE_CONE_MID_ENC_POS;
                    break;
                    
                case COMMAND_STATE_SCORE_HIGH_CONE :
                    targetPositionEnc = SCORE_CONE_HIGH_ENC_POS;
                    break;
                    
                case  COMMAND_STATE_DO_NOTHING :
                     break;

                default:
                    pidEnable = false;
                    break;
            }
        }

        if(killPID){                //TBD - Right precedence?
            pidEnable = false;
        }

        if(softLimitOverride){
            wristMtr.configForwardSoftLimitEnable(false);
            wristMtr.configReverseSoftLimitEnable(false);
        }
        else{
            //wristMtr.configForwardSoftLimitEnable(true);
            //wristMtr.configReverseSoftLimitEnable(true);
        }
    }



    public void wristManual(double pwr)
    {
        mtrPower = pwr * WRIST_MAX_PWR;
        wristMtr.set(ControlMode.PercentOutput, mtrPower);
    }

    public void rollersIn()
    {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR);
    }

    public void rollersOut()
    {
        rollersMtr.set(ControlMode.PercentOutput, -ROLLERS_PWR);
    }

    public void rollersOff()
    {
        rollersMtr.set(ControlMode.PercentOutput, 0.0);
    }
    
    public double calcWristAngle()//testing
    {
        double wristAngle = (wristMtr.getSelectedSensorPosition() - WRIST_ENC_OFFSET) * WRIST_DEGREE_PER_CNT;
        return wristAngle;
    }
    
    public double calculateGravityFF()
    {
        double radians = Math.toRadians(calcWristAngle() - CM_DEGREE_OFFSET);
        double cosineScalar = Math.cos(radians);
        
        return maxGravityFF * cosineScalar;
    }

    public void shuffleboardIntake()
    {
    
    }

    public void shuffleboardIntakeDebug()
    {
        SmartDashboard.putNumber("motor remote sensor postion", wristMtr.getSelectedSensorPosition() - WRIST_ENC_OFFSET);
        SmartDashboard.putNumber("calculated ang", calcWristAngle() - CM_DEGREE_OFFSET);
        SmartDashboard.putNumber("GravityFF", calculateGravityFF());
        SmartDashboard.putNumber("CLosedLoopError", pid.getPositionError());
        SmartDashboard.putNumber("applied output", wristMtr.getMotorOutputPercent() );
        SmartDashboard.putBoolean("pid", pidEnable);
    }
}