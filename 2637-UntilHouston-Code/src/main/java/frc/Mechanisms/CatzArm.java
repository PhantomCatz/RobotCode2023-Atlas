package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

@SuppressWarnings("unused")
public class CatzArm
{
    private WPI_TalonFX armMtr;

    private final int ARM_MC_ID = 20;

    private final double EXTEND_PWR  = 0.2;
    private final double RETRACT_PWR = -0.2;

    //Conversion factors

    //current limiting
    private SupplyCurrentLimitConfiguration armCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    //gear ratio
    private final double VERSA_RATIO  = 7.0/1.0;

    private final double PUILEY_1      = 24.0;
    private final double PUILEY_2      = 18.0;
    private final double PUILEY_RATIO  = PUILEY_1 / PUILEY_2;
     
    private final double FINAL_RATIO   = VERSA_RATIO * PUILEY_RATIO;
    private final double FINAL_CIRCUMFERENCE = 3.54; 


    private final boolean LIMIT_SWITCH_IGNORED = false;
    private final boolean LIMIT_SWITCH_MONITORED = true;

    private final double CNTS_OVER_REV = 2048.0 / 1.0;

    private final double CNTS_PER_INCH_CONVERSION_FACTOR = CNTS_OVER_REV/FINAL_CIRCUMFERENCE;

    private final double POS_ENC_INCH_RETRACT = 0.0;
    private final double POS_ENC_INCH_EXTEND = 8.157;
    private final double POS_ENC_INCH_PICKUP = 4.157;

    private final double POS_ENC_CNTS_RETRACT  = POS_ENC_INCH_RETRACT * CNTS_PER_INCH_CONVERSION_FACTOR;
    private final double POS_ENC_CNTS_EXTEND  = 44000.0;//POS_ENC_INCH_EXTEND * CNTS_PER_INCH_CONVERSION_FACTOR;
    private final double POS_ENC_CNTS_PICKUP = POS_ENC_INCH_PICKUP * CNTS_PER_INCH_CONVERSION_FACTOR;


    private boolean extendSwitchState = false;

    private int SWITCH_CLOSED = 1;

    private final double ARM_KP = 0.1;
    private final double ARM_KI = 0.0;
    private final double ARM_KD = 0.0;

    private final double ARM_CLOSELOOP_ERROR = 3000;

    private final double MANUAL_CONTROL_PWR_OFF = 0.0;

    public CatzArm()
    {
        armMtr = new WPI_TalonFX(ARM_MC_ID);

        armMtr.configFactoryDefault();

        armMtr.setNeutralMode(NeutralMode.Brake);
        armMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        armMtr.overrideLimitSwitchesEnable(LIMIT_SWITCH_MONITORED);

        armMtr.config_kP(0, ARM_KP);
        armMtr.config_kI(0, ARM_KI);
        armMtr.config_kD(0, ARM_KD);

        armCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        armMtr.configSupplyCurrentLimit(armCurrentLimit);

        armMtr.configAllowableClosedloopError(0, ARM_CLOSELOOP_ERROR);

        armMtr.set(ControlMode.PercentOutput, MANUAL_CONTROL_PWR_OFF);

    }

    public void cmdProcArm(boolean armExtend, boolean armRetract,
                            int cmdState)
    {
        

        if(cmdState == Robot.COMMAND_STATE_PICKUP_CONE||
           cmdState == Robot.COMMAND_STATE_PICKUP_CUBE ||
           cmdState == Robot.COMMAND_STATE_SCORE_LOW_CONE ||
           cmdState == Robot.COMMAND_STATE_SCORE_LOW_CUBE)
            {
                armMtr.set(ControlMode.Position, 22000);
            }
        else if(cmdState == Robot.COMMAND_STATE_SCORE_HIGH_CONE||
                cmdState == Robot.COMMAND_STATE_SCORE_HIGH_CUBE )
            {
                armMtr.set(ControlMode.Position, POS_ENC_CNTS_EXTEND);
            }
        else if(cmdState == Robot.COMMAND_STATE_STOW ||
                cmdState == Robot.COMMAND_STATE_SCORE_MID_CONE ||
                cmdState == Robot.COMMAND_STATE_SCORE_MID_CUBE)
            {
                System.out.println("inside retract block");
                armMtr.set(ControlMode.Position, POS_ENC_CNTS_RETRACT);
            }


        if(armExtend == true)
        {

            Robot.commandedState = Robot.COMMAND_STATE_DO_NOTHING;
            setArmPwr(EXTEND_PWR);
        }
        else if(armRetract == true)
        {
            Robot.commandedState = Robot.COMMAND_STATE_DO_NOTHING;
            setArmPwr(RETRACT_PWR);
        }
        else if(armMtr.getControlMode() == ControlMode.PercentOutput)
        {
            setArmPwr(MANUAL_CONTROL_PWR_OFF);
        }
        
    }


    public void checkLimitSwitches()
    {
        if(armMtr.getSensorCollection().isRevLimitSwitchClosed() == SWITCH_CLOSED)
        {
            armMtr.setSelectedSensorPosition(POS_ENC_CNTS_RETRACT);
            extendSwitchState = true;
        }
        else
        {
            extendSwitchState = false;
        }


    }

    public void setArmPwr(double pwr)
    {        
        armMtr.set(ControlMode.PercentOutput, pwr);
    }

    public void smartDashboardARM()
    {
        SmartDashboard.putNumber("encoder position", armMtr.getSelectedSensorPosition());
    }
    
}
