package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import frc.DataLogger.*;;

public class CatzIntake
{
    private WPI_TalonFX wristMtr;
    private WPI_TalonFX rollersMtr;

    private final double THREAD_PERIOD = 0.02;

    private final double SOFT_LIMIT_FORWARD = 3887.0;
    private final double SOFT_LIMIT_REVERSE = -1787;

    private final double SEMI_MANUAL_RATE = 50.0;

    private final int WRIST_MC_ID   = 31;
    private final int ROLLERS_MC_ID = 30;

    private final double WRIST_MAX_PWR = 0.3;
    private final double ROLLERS_PWR   = 0.4;
    
    private final int WRIST_ENC_CAN_ID = 13; 

    private final double WRIST_ENC_OFFSET = -989;

    private final double ENC_TO_INTAKE_GEAR_RATIO =  18.0/46.0;
    private final double WRIST_DEGREE_PER_CNT = 360.0 / 4096.0 * ENC_TO_INTAKE_GEAR_RATIO;

    private final double STOW_ENC_POS = 3883.0;
    private final double INTAKE_CUBE_ENC_POS = -335; // TBD
    private final double INTAKE_CONE_ENC_POS_GROUND = -1295; //TBD
    private final double INTAKE_CONE_ENC_POS_SINGLE = 1100;
    private final double SCORE_CUBE_ENC_POS = 870.0; // TBD
    private final double SCORE_CONE_HIGH_ENC_POS = -300;//-700;//-1123.0; //TBD
    private final double SCORE_CONE_MID_ENC_POS = INTAKE_CONE_ENC_POS_GROUND; //TBD
    private final double SCORE_CONE_LOW_ENC_POS = INTAKE_CONE_ENC_POS_GROUND; //TBD

    private final double STOW_CUTOFF = 3670.0;

    public final int GP_NULL = 0;
    public final int GP_CUBE = 1;
    public final static int GP_CONE = 2;

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

    private final double kP = 0.00009; 
    private final double kI = 0.000040;
    private final double kD = 0.000007;
    
    double maxGravityFF = 0.11; //0.11
    
    private PIDController pid;
    private double targetPositionEnc = STOW_ENC_POS;
    private double targetPower;
    private Boolean pidEnable = false;

    private double mtrPower;
    private CANCoder wristEncoder;

    private double wristManualThreadPower = 0.0;

    private int intakeMovementMode = Robot.MODE_AUTO;

    private double targetPositionAuton = -999999.0;
    private double currentPosition = -999.0;
    private double positionError = -999.0; 

    private boolean intakeInPosition = false;

    private final double ERROR_INTAKE_THRESHOLD = 500;
    private final double NO_TARGET_POSITION = -999999.0;


    CatzLog data;

    public CatzIntake()
    {
        wristEncoder = new CANCoder(WRIST_ENC_CAN_ID);//do we use?

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
                    targetPower = pid.calculate(wristMtr.getSelectedSensorPosition(), targetPositionEnc) + calculateGravityFF();
                    if(targetPositionEnc == STOW_ENC_POS && wristMtr.getSelectedSensorPosition() > STOW_CUTOFF){
                        targetPower = 0.0;
                    }
                    wristMtr.set(ControlMode.PercentOutput, targetPower);
                }

                currentPosition = wristMtr.getSelectedSensorPosition();
                positionError = currentPosition - targetPositionAuton;
                if  ((Math.abs(positionError) <= ERROR_INTAKE_THRESHOLD) && targetPositionAuton != NO_TARGET_POSITION)
                {
                    intakeInPosition = true;
                    targetPositionAuton = NO_TARGET_POSITION;
                }

                Timer.delay(THREAD_PERIOD);    
            }   //End of while(true)
        });
        intakeThread.start();
    }
    


    public void cmdProcIntake(double wristPwr, boolean rollersIn, boolean rollersOut, boolean manualMode, boolean softLimitOverride, int state, int gamePiece)
    {
        if(manualMode){                
            pidEnable = false;
            intakeMovementMode = Robot.MODE_MANUAL;
        }

        if(Math.abs(wristPwr) >= 0.1)
        {
            if (pidEnable == true)//in manual holding state
            {
                intakeMovementMode = Robot.MODE_MANUAL_HOLD;

                if(wristPwr > 0){
                    targetPositionEnc = Math.min((targetPositionEnc + wristPwr * SEMI_MANUAL_RATE), SOFT_LIMIT_FORWARD);
                }
                else{
                    targetPositionEnc = Math.max((targetPositionEnc + wristPwr * SEMI_MANUAL_RATE), SOFT_LIMIT_REVERSE);
                }
            }
            else//in full manual mode
            {
                wristManualThreadPower = wristPwr * WRIST_MAX_PWR;    
                wristMtr.set(ControlMode.PercentOutput, wristManualThreadPower);
            }
        }
        else if(pidEnable == false)
        {
            wristManualThreadPower = 0.0;
        }


        if(rollersIn)
        {
            if(gamePiece == GP_CUBE)
            {
                rollersInCube();
            } 
            else 
            {
                rollersInCone();
            }
        }
        else if(rollersOut)
        {
            if(gamePiece == GP_CUBE)
            {
                rollersOutCube();
            } 
            else 
            {
                rollersOutCone();
            }
        }
        else
        {
            rollersOff();
        }

        

        if(state != Robot.COMMAND_STATE_NULL)
        {
            pid.reset();
             pidEnable = true;
             intakeMovementMode = Robot.MODE_AUTO;
             intakeInPosition = false;
            switch(state)
            {
                case Robot.COMMAND_UPDATE_STOW :
                    targetPositionEnc = STOW_ENC_POS;
                    break;
                    
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE :
                    targetPositionEnc = INTAKE_CONE_ENC_POS_GROUND;
                    break;
                    
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE :
                    targetPositionEnc = INTAKE_CUBE_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE :
                    targetPositionEnc = INTAKE_CONE_ENC_POS_SINGLE;
                    break;
                    
                case Robot.COMMAND_UPDATE_SCORE_LOW_CONE :
                    targetPositionEnc = SCORE_CONE_LOW_ENC_POS;
                    break;
    
                case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE :
                    targetPositionEnc = SCORE_CUBE_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_SCORE_MID_CUBE :
                    targetPositionEnc = SCORE_CUBE_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
                    targetPositionEnc = SCORE_CUBE_ENC_POS;
                    break;
    
                case Robot.COMMAND_UPDATE_SCORE_MID_CONE :
                    targetPositionEnc = SCORE_CONE_MID_ENC_POS;
                    break;
                    
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE :
                    targetPositionEnc = SCORE_CONE_HIGH_ENC_POS;
                    break;

                default:
                    pidEnable = false;
                    break;
            }

            targetPositionAuton = targetPositionEnc;

            if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE))
            {        
                data = new CatzLog(Robot.currentTime.get(), wristMtr.getSelectedSensorPosition(), targetPower, 
                                                            targetPositionEnc, 
                                                            calculateGravityFF(), 
                                                                        -999.0, 
                                                                        -999.0,
                                                                        -999.0, -999.0, -999.0, 
                                                                        -999.0, -999.0, -999.0, -999.0, -999.0,
                                                                        DataCollection.boolData);  
                Robot.dataCollection.logData.add(data);
            }
        }

        if(softLimitOverride){
            wristMtr.configForwardSoftLimitEnable(false);
            wristMtr.configReverseSoftLimitEnable(false);
        }
        else{
            wristMtr.configForwardSoftLimitEnable(true);
            wristMtr.configReverseSoftLimitEnable(true);
        }
    }



    public void wristManual(double pwr)
    {
        mtrPower = pwr * WRIST_MAX_PWR;
        wristMtr.set(ControlMode.PercentOutput, mtrPower);
    }

    public void rollersOff()
    {
        rollersMtr.set(ControlMode.PercentOutput, 0.0);
    }

    public void rollersInCube()
    {
        rollersMtr.set(ControlMode.PercentOutput, -ROLLERS_PWR);
    }

    public void rollersOutCube()
    {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR);
    }

    public void rollersInCone()
    {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR);
    }

    public void rollersOutCone()
    {
        rollersMtr.set(ControlMode.PercentOutput, -ROLLERS_PWR);
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

    public int getIntakeMovementMode()
    {
        return intakeMovementMode;
    }

    public void shuffleboardIntake()
    {
    
    }

    public void smartdashboardIntakeDebug()
    {
        SmartDashboard.putNumber("motor remote sensor postion", wristMtr.getSelectedSensorPosition());
        SmartDashboard.putNumber("calculated ang", calcWristAngle());
        SmartDashboard.putNumber("GravityFF", calculateGravityFF());
        SmartDashboard.putNumber("CLosedLoopError", pid.getPositionError());
        SmartDashboard.putNumber("applied output", wristMtr.getMotorOutputPercent() );
        SmartDashboard.putBoolean("pid", pidEnable);
    }

    public double intakeWristTemp()
    {
        return wristMtr.getTemperature();
    }

    public boolean isIntakeInPos()
    {
        return intakeInPosition;
    }
}
