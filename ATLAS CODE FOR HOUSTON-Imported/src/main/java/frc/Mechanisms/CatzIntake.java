package frc.Mechanisms;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Robot.mechMode;
import frc.DataLogger.*;

public class CatzIntake {
    // ----------------------------------------------------------------------------------------------
    //
    // Roller
    //
    // ----------------------------------------------------------------------------------------------
    private WPI_TalonFX rollersMtr;

    private final int ROLLERS_MC_ID = 30;

    private final double ROLLERS_PWR_CUBE_IN = -0.8;
    private final double ROLLERS_PWR_CONE_IN = 1.0; // TBD decide pwrs for all cube cone scoring rollers

    private final double ROLLERS_PWR_CUBE_OUT = 1.0;
    private final double ROLLERS_PWR_CONE_OUT = -0.5;

    private SupplyCurrentLimitConfiguration rollerCurrentLimit;

    private final int CURRENT_LIMIT_AMPS_ROLLER = 40;
    private final int CURRENT_LIMIT_TRIGGER_AMPS_ROLLER = 40;

    // ----------------------------------------------------------------------------------------------
    //
    // Wrist
    //
    // ----------------------------------------------------------------------------------------------
    private WPI_TalonFX wristMtr;

    private final int WRIST_MC_ID = 31;

    private final double WRIST_MAX_PWR = 0.3;

    // current limiting
    private SupplyCurrentLimitConfiguration wristCurrentLimit;

    private final int CURRENT_LIMIT_AMPS = 55;
    private final int CURRENT_LIMIT_TRIGGER_AMPS = 55;
    private final double CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT = true;

    // ----------------------------------------------------------------------------------------------
    // Wrist encoder & Position Values
    // ----------------------------------------------------------------------------------------------

    private final double ENC_TO_INTAKE_GEAR_RATIO = (46.0 / 18.0)* (32.0 / 10.0);
    private final double WRIST_CNTS_PER_DEGREE = (2096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;

    private final double MANUAL_HOLD_STEP_SIZE = 2.0;

    // TBD - ADD comment for ref point
    private final double CENTER_OF_MASS_OFFSET_DEG = 177.0;
    private final double WRIST_ABS_ENC_OFFSET_DEG = 0.0; // Set to make stow pos equal to 0
    private final double WRIST_ABS_ENC_OFFSET = WRIST_ABS_ENC_OFFSET_DEG * WRIST_CNTS_PER_DEGREE;// -989.0; //Negative
                                                                                                 // value means abs enc
                                                                                                 // 0 is above intake
                                                                                                 // angle 0

    private final double STOW_ENC_POS = -20.0 + WRIST_ABS_ENC_OFFSET_DEG;// 4872.0 + WRIST_ABS_ENC_OFFSET; //3883
    private final double STOW_CUTOFF = -30.232 + WRIST_ABS_ENC_OFFSET_DEG;// + WRIST_ABS_ENC_OFFSET; //3670

    private final double INTAKE_CUBE_ENC_POS = -155.000 + WRIST_ABS_ENC_OFFSET_DEG;// 1324.0 + WRIST_ABS_ENC_OFFSET;
                                                                                   // //-335
    private final double INTAKE_CONE_ENC_POS_GROUND = -170.524 + WRIST_ABS_ENC_OFFSET_DEG;// -306.0 +
                                                                                          // WRIST_ABS_ENC_OFFSET;
                                                                                          // //-1295
    private final double INTAKE_CONE_ENC_POS_SINGLE = -100.400 + WRIST_ABS_ENC_OFFSET_DEG;// 2089.0 +
                                                                                          // WRIST_ABS_ENC_OFFSET;
                                                                                          // //1100

    private final double SCORE_CUBE_ENC_POS = -90.000 + WRIST_ABS_ENC_OFFSET_DEG;// //prev -80
                                                                                 // // Applies to low-mid-high

    private final double SCORE_CONE_HIGH_ENC_POS = -140.000 + WRIST_ABS_ENC_OFFSET_DEG;// 289.0 + WRIST_ABS_ENC_OFFSET;
                                                                                       // //-700
    private final double SCORE_CONE_MID_ENC_POS = -170.000;//INTAKE_CONE_ENC_POS_GROUND; // TBD verify if its the same as high
    private final double SCORE_CONE_LOW_ENC_POS = -130.00;//INTAKE_CONE_ENC_POS_GROUND; // TBD

    private final double SOFT_LIMIT_FORWARD = -160.0; // 4876 + WRIST_ABS_ENC_OFFSET; //3887
    private final double SOFT_LIMIT_REVERSE = -8900.0; // -798.0 + WRIST_ABS_ENC_OFFSET; //-1787 //TBD

    private final double GROSS_kP = 0.002472;// 0.0070;//0.00009;
    private final double GROSS_kI = 0.0;// 000040;
    private final double GROSS_kD = 0.000291;// 0.000007;

    private final double FINE_kP = 0.005234;// 0.00009;
    private final double FINE_kI = 0.0;// 000008;
    private final double FINE_kD = 0.000291;// 0.000291;//0.000007;

    private final double MAX_GRAVITY_FF = 0.07; // 0.055

    private PIDController pid;

    private Boolean pidEnable = false;

    private double targetPositionDeg = STOW_ENC_POS;

    private double targetPower = 0.0;
    private double prevTargetPwr = 0.0;

    private int intakeMovementMode = Robot.MODE_AUTO; // TBD - Should we have NULL State?

    private double currentPositionDeg = -999.0;
    private double positionError = -999.0;
    private double prevCurrentPosition = -999.0;

    private boolean intakeInPosition = false;

    private final double ERROR_INTAKE_THRESHOLD_DEG = 5.0;
    private final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;

    private double pidPower = 0.0;
    private double ffPower = 0.0;

    private int numConsectSamples = 0;

    // private double wristAngle;

    /*----------------------------------------------------------------------------------------------
    *
    *  Misc
    *
    *---------------------------------------------------------------------------------------------*/
    CatzLog data;

    private final double THREAD_PERIOD = 0.02;

    private int cmdstateUpdate;

    /*----------------------------------------------------------------------------------------------
    *
    *  CatzIntake()
    *
    *---------------------------------------------------------------------------------------------*/
    public CatzIntake() {
        // ----------------------------------------------------------------------------------------------
        // Roller
        // ----------------------------------------------------------------------------------------------
        rollersMtr = new WPI_TalonFX(ROLLERS_MC_ID);
        rollersMtr.configFactoryDefault();
        rollersMtr.setNeutralMode(NeutralMode.Brake);

        rollerCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS_ROLLER,
                CURRENT_LIMIT_TRIGGER_AMPS_ROLLER,
                CURRENT_LIMIT_TIMEOUT_SECONDS);

        rollersMtr.configSupplyCurrentLimit(rollerCurrentLimit);

        // ----------------------------------------------------------------------------------------------
        // Wrist
        // ----------------------------------------------------------------------------------------------
        wristMtr = new WPI_TalonFX(WRIST_MC_ID);

        wristMtr.configFactoryDefault();

        wristMtr.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        // wristMtr.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
        wristMtr.configIntegratedSensorOffset(0.0);

        wristMtr.setNeutralMode(NeutralMode.Brake);

        wristMtr.configForwardSoftLimitThreshold(SOFT_LIMIT_FORWARD);
        wristMtr.configReverseSoftLimitThreshold(SOFT_LIMIT_REVERSE);

        wristMtr.configForwardSoftLimitEnable(true);
        wristMtr.configReverseSoftLimitEnable(true);

        wristCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS,
                                                                CURRENT_LIMIT_TRIGGER_AMPS,
                                                                CURRENT_LIMIT_TIMEOUT_SECONDS);
        wristMtr.configSupplyCurrentLimit(wristCurrentLimit);
        wristMtr.configOpenloopRamp(0);
        pid = new PIDController(GROSS_kP, GROSS_kI, GROSS_kD);

        startIntakeThread();
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  startIntakeThread()
    *
    *---------------------------------------------------------------------------------------------*/
    public void startIntakeThread() {
        Thread intakeThread = new Thread(() -> {
            while (true) {

                if (pidEnable) {
                    // ----------------------------------------------------------------------------------
                    // Chk if at final position
                    // ----------------------------------------------------------------------------------
                    currentPositionDeg = wristMtr.getSelectedSensorPosition() / WRIST_CNTS_PER_DEGREE;

                    positionError = currentPositionDeg - targetPositionDeg;
                    
                    if ((Math.abs(positionError) <= ERROR_INTAKE_THRESHOLD_DEG)) {
                        numConsectSamples++;
                        if (numConsectSamples >= 1) {
                            intakeInPosition = true;
                        }
                    } else {
                        numConsectSamples = 0;
                    }

                    if (Math.abs(positionError) >= PID_FINE_GROSS_THRESHOLD_DEG) {
                        pid.setP(GROSS_kP);
                        pid.setI(GROSS_kI);
                        pid.setD(GROSS_kD);

                    } else if (Math.abs(positionError) < PID_FINE_GROSS_THRESHOLD_DEG) {
                        pid.setP(FINE_kP);
                        pid.setI(FINE_kI);
                        pid.setD(FINE_kD);
                    }

                    pidPower = pid.calculate(currentPositionDeg, targetPositionDeg);
                    ffPower = calculateGravityFF();
                    targetPower = pidPower + ffPower;

                    // -------------------------------------------------------------
                    // checking if we did not get updated position value(Sampling Issue).
                    // If no change in position, this give invalid target power(kD issue).
                    // Therefore, go with prev targetPower Value.
                    // -------------------------------------------------------------------
                    if (prevCurrentPosition == currentPositionDeg) {
                        targetPower = prevTargetPwr;
                    }

                    // ----------------------------------------------------------------------------------
                    // If we are going to Stow Position & have passed the power cutoff angle, set
                    // power to 0, otherwise calculate new motor power based on position error and
                    // current angle
                    // ----------------------------------------------------------------------------------
                    if (targetPositionDeg == STOW_ENC_POS && currentPositionDeg > STOW_CUTOFF) {
                        targetPower = 0.0;
                    }
                    wristMtr.set(ControlMode.PercentOutput, targetPower);

                    prevCurrentPosition = currentPositionDeg;
                    prevTargetPwr = targetPower;

                }

                if ((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE)) {
                    data = new CatzLog(Robot.currentTime.get(), targetPositionDeg, currentPositionDeg,
                            targetPower,
                            pidPower,
                            ffPower,
                            wristMtr.getMotorOutputPercent(),
                            wristMtr.getStatorCurrent(), wristMtr.getSupplyCurrent(), -999.0,
                            -999.0, -999.0, -999.0, -999.0, -999.0,
                            DataCollection.boolData);

                    Robot.dataCollection.logData.add(data);
                }

                Timer.delay(THREAD_PERIOD);
            } // End of while(true)
        });
        intakeThread.start();
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  cmdProcIntake()
    *
    *---------------------------------------------------------------------------------------------*/
    public void cmdProcIntake(double wristPwr, boolean rollersIn, boolean rollersOut, boolean manualMode,
                                                                                        boolean softLimitOverride,
                                                                                        int state,
                                                                                        int gamePiece) {
        if (manualMode) {
            pidEnable = false;
            intakeMovementMode = Robot.MODE_MANUAL;
            Robot.intakeControlMode = mechMode.ManualMode;
        }
        

        if (Math.abs(wristPwr) >= 0.1) {
            if (pidEnable == true)// in manual holding state
            {
                Robot.intakeControlMode = mechMode.ManualHoldMode;

                if (wristPwr > 0) {
                    targetPositionDeg = Math.min((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE),
                            -30);
                } else {
                    targetPositionDeg = Math.max((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE),
                            -180);
                }
                prevCurrentPosition = -prevCurrentPosition; // intialize for first time through thread loop, that checks
                                                            // stale position values

            } else // in full manual mode
            {
                targetPower = wristPwr * WRIST_MAX_PWR;
                wristMtr.set(ControlMode.PercentOutput, targetPower);
            }
        } else // Manual power is OFF
        {
            if (pidEnable == false) {
                targetPower = 0.0;
                wristMtr.set(ControlMode.PercentOutput, targetPower);
            }
        }

        if (rollersIn) {
            if (gamePiece == Robot.GP_CUBE) {
                rollersInCube();
            } else {
                rollersInCone();
            }
        } else if (rollersOut) {
            if (gamePiece == Robot.GP_CUBE) {
                rollersOutCube();
            } else {
                rollersOutCone();
            }
        } else {
            rollersOff();
        }

        if (state != Robot.COMMAND_STATE_NULL) {
            pid.reset();
            pidEnable = true;
            intakeInPosition = false;
            Robot.intakeControlMode = mechMode.AutoMode;
            prevCurrentPosition = -prevCurrentPosition; // intialize for first time through thread loop, that checks
                                                        // stale position values

            switch (state) {
                case Robot.COMMAND_UPDATE_STOW:
                    targetPositionDeg = STOW_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE:
                    targetPositionDeg = INTAKE_CONE_ENC_POS_GROUND;
                    break;

                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE:
                    targetPositionDeg = INTAKE_CUBE_ENC_POS;
                    System.out.println("he");
                    break;

                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE:
                    targetPositionDeg = INTAKE_CONE_ENC_POS_SINGLE;
                    break;

                case Robot.COMMAND_UPDATE_SCORE_LOW_CONE:
                    targetPositionDeg = SCORE_CONE_LOW_ENC_POS;
                    Timer.delay(0.1);// wait for arm to deploy
                    break;

                case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE:
                case Robot.COMMAND_UPDATE_SCORE_MID_CUBE:
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
                    targetPositionDeg = SCORE_CUBE_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_SCORE_MID_CONE:
                    targetPositionDeg = SCORE_CONE_MID_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE:
                    targetPositionDeg = SCORE_CONE_HIGH_ENC_POS;
                    break;

                default:
                    pidEnable = false;
                    // TBD
                    break;
            }
        }

        if (softLimitOverride) {
            wristMtr.configForwardSoftLimitEnable(false);
            wristMtr.configReverseSoftLimitEnable(false);
        } else {
            wristMtr.configForwardSoftLimitEnable(true);
            wristMtr.configReverseSoftLimitEnable(true);
        }
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - PID 
    *
    *---------------------------------------------------------------------------------------------*/
    public void resetPID() {
        pidEnable = false;
        pid.reset();
    }

    public void enablePID(boolean set) {
        pidEnable = set;
    }

    public boolean getPIDEnabled() {
        return pidEnable;
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Rollers
    *
    *---------------------------------------------------------------------------------------------*/
    public void rollersOff() {
        rollersMtr.set(ControlMode.PercentOutput, 0.0);
    }

    public void rollersInCube() {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CUBE_IN);
    }

    public void rollersOutCube() {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CUBE_OUT);
    }

    public void rollersInCone() {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CONE_IN);
    }

    public void rollersOutCone() {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CONE_OUT);
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Wrist
    *
    *---------------------------------------------------------------------------------------------*/
    public double calcWristAngle() {
        double wristAngle = ((wristMtr.getSelectedSensorPosition() / WRIST_CNTS_PER_DEGREE) - WRIST_ABS_ENC_OFFSET_DEG);
        return wristAngle;
    }

    public double calculateGravityFF() {
        double radians = Math.toRadians(calcWristAngle() - CENTER_OF_MASS_OFFSET_DEG);
        double cosineScalar = Math.cos(radians);

        return MAX_GRAVITY_FF * cosineScalar;
    }

    public double intakeWristTemp() {
        return wristMtr.getTemperature();
    }

    public int getIntakeMovementMode() {
        return intakeMovementMode;
    }

    public void shuffleboardIntake() {

    }

    public void smartdashboardIntakeDebug() {
        SmartDashboard.putNumber("wrist ang", calcWristAngle());
        SmartDashboard.putNumber("GravityFF", calculateGravityFF());
        SmartDashboard.putNumber("IntakeClosedLoopError", pid.getPositionError());
        SmartDashboard.putNumber("applied output", wristMtr.getMotorOutputPercent());
        SmartDashboard.putBoolean("pid", pidEnable);
        SmartDashboard.putNumber("mtr abs", wristMtr.getSelectedSensorPosition());
    }

    public boolean isIntakeInPos() {
        return intakeInPosition;
    }

}