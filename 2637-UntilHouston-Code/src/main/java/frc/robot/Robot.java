// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import java.util.ArrayList;

import javax.lang.model.util.ElementScanner14;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;

import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzElevator;
import frc.Mechanisms.CatzArm;
import frc.Mechanisms.CatzIntake;
import frc.Autonomous.CatzAutonomous;
import frc.Autonomous.CatzAutonomousPaths;
import frc.Autonomous.CatzBalance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

 @SuppressWarnings("unused")
public class Robot extends TimedRobot 
{
  //---------------------------------------------------------------------------------------------
  //  Shared Libraries & Utilities
  //---------------------------------------------------------------------------------------------
  public static CatzConstants       constants;

  public static DataCollection      dataCollection;
  public ArrayList<CatzLog>         dataArrayList;

  //----------------------------------------------------------------------------------------------
  //  Shared Robot Components (e.g. not mechanism specific, such as PDH, NavX, etc)
  //----------------------------------------------------------------------------------------------
  public static PowerDistribution PDH;

  public static AHRS              navX;

  public static Timer             currentTime;

  private XboxController xboxDrv;
  private XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  public static final int DPAD_UP = 0;
  public static final int DPAD_DN = 180;
  public static final int DPAD_LT = 270;
  public static final int DPAD_RT = 90;

  public int xboxGamePieceSelection = DPAD_RT;
  public double  xboxManual    = 0.1;
  public boolean xboxStowPos   = false;
  public boolean xboxPickUpPos = false;
  public boolean xboxLowNode   = false;
  public boolean xboxMidNode   = false;
  public boolean xboxHighNode  = false;

  public static final int COMMAND_STATE_STOW            = 0;

  public static final int COMMAND_STATE_PICKUP_CONE     = 1;
  public static final int COMMAND_STATE_PICKUP_CUBE     = 2;

  public static final int COMMAND_STATE_SCORE_LOW_CONE  = 10;
  public static final int COMMAND_STATE_SCORE_LOW_CUBE  = 11;
  public static final int COMMAND_STATE_SCORE_MID_CONE  = 12;
  public static final int COMMAND_STATE_SCORE_MID_CUBE  = 13;
  public static final int COMMAND_STATE_SCORE_HIGH_CONE = 14;
  public static final int COMMAND_STATE_SCORE_HIGH_CUBE = 15;

  public static final int COMMAND_STATE_DO_NOTHING          = 20;

  public static int commandedState = COMMAND_STATE_DO_NOTHING;

  public final int GP_CUBE = 0;
  public final int GP_CONE = 1;
  public final int GP_NONE = 2;

  public int selectedGamePiece = GP_CUBE;

  //public boolean stateChosen = false;     //TBD
    
  
  //---------------------------------------------------------------------------------------------
  //  Autonomous
  //---------------------------------------------------------------------------------------------
  public static CatzAutonomous      auton;
  public static CatzAutonomousPaths paths;
  public static CatzBalance         balance;

  private final double OFFSET_DELAY = 0.5;    //TBD put into AUTO BALANCE class

  //---------------------------------------------------------------------------------------------
  //  Mechanisms
  //---------------------------------------------------------------------------------------------
  public static CatzDrivetrain drivetrain;
  public static  CatzElevator  elevator;
  public static  CatzArm       arm;
  public static CatzIntake     intake;


  /*-----------------------------------------------------------------------------------------
  *  
  *  robotXxx
  *
  *----------------------------------------------------------------------------------------*/
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    //-----------------------------------------------------------------------------------------
    //  Shared Libraries & Utilities
    //-----------------------------------------------------------------------------------------
    constants      = new CatzConstants();

    dataCollection = new DataCollection();
    dataArrayList  = new ArrayList<CatzLog>();
    
    dataCollection.dataCollectionInit(dataArrayList);


    //-----------------------------------------------------------------------------------------
    //  Shared Robot Components (e.g. not mechanism specific)
    //-----------------------------------------------------------------------------------------
    PDH = new PowerDistribution();

    navX = new AHRS();
    navX.reset();

    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    currentTime = new Timer();

    currentTime.reset();
    currentTime.start();

    //----------------------------------------------------------------------------------------------
    //  Autonomous
    //----------------------------------------------------------------------------------------------
    auton   = new CatzAutonomous();
    paths   = new CatzAutonomousPaths();
    balance = new CatzBalance();

    //----------------------------------------------------------------------------------------------
    //  Mechanisms
    //----------------------------------------------------------------------------------------------
    drivetrain = new CatzDrivetrain();
    elevator   = new CatzElevator();
    arm        = new CatzArm();
    intake     = new CatzIntake();

    //led        = new CatzRGB();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    //----------------------------------------------------------------------------------------------
    //  Update status, LED's
    //----------------------------------------------------------------------------------------------
    elevator.checkLimitSwitches();

    dataCollection.updateLogDataID(); //NEEDS TO BE FIXED..causes robot to crash

    //led.LEDWork();

    //----------------------------------------------------------------------------------------------
    //  Shuffleboard Data Display
    //----------------------------------------------------------------------------------------------
    SmartDashboard.putNumber("NavX", navX.getAngle());

    drivetrain.smartDashboardDriveTrain();
      elevator.smartDashboardElevator();
          
      /*arm.smartDashboardArm();
        intake.smartDashboardIntake();
       balance.SmartDashboardBalance();
    
    //debug should be commented out for comp
    drivetrain.smartDashboardDriveTrain_DEBUG();
      elevator.smartDashboardElevator_DEBUG();*/
        intake.shuffleboardIntakeDebug();
       //balance.SmartDashboardBalanceDebug();
       
  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  autonomousXxx
  *
  *----------------------------------------------------------------------------------------*/
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    intake.resetPID();

    currentTime.reset();
    currentTime.start();

    navX.reset();

    navX.setAngleAdjustment(-navX.getYaw() + 180.0); //set navx's zero position to opposite way robot is facing
    
    Timer.delay(OFFSET_DELAY);  //TBD - This should be 

 //   paths.executeSelectedPath();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {
    
  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  teleopXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit()
  {
    intake.resetPID();
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
  {
    if(xboxAux.getPOV() == DPAD_RT){
      xboxGamePieceSelection = GP_CUBE;
    }
    else if(xboxAux.getPOV() == DPAD_LT){
      xboxGamePieceSelection = GP_CONE;
    }
    else if(xboxAux.getBackButtonPressed()){
      xboxGamePieceSelection = GP_NONE;
    }
    //not updated to newest controller map
    xboxManual    = xboxAux.getRightY();
    xboxHighNode  = xboxAux.getYButton();
    xboxMidNode   = xboxAux.getBButton();
    xboxLowNode   = xboxAux.getAButton();
    xboxStowPos   = xboxAux.getStartButtonPressed();
    xboxPickUpPos = xboxAux.getBackButtonPressed(); //conflict w piddisable
    
    determineCommandState(xboxGamePieceSelection, xboxLowNode, 
                                                  xboxMidNode, 
                                                  xboxHighNode, xboxStowPos, xboxPickUpPos);
  
    elevator.cmdProcElevator(xboxManual,                           // Manual / Manual Hold Elevator Power
                             xboxAux.getRightStickButtonPressed(), // Enter all-manual mode
                             commandedState);

    arm.cmdProcArm(xboxAux.getPOV() == DPAD_UP,   //Manual Extend Arm
                   xboxAux.getPOV() == DPAD_DN,   //Manual Retract Arm 
                   commandedState); 

    intake.cmdProcIntake(
      -xboxAux.getLeftY(),                       //Semi-manual override
      xboxAux.getRightTriggerAxis() >= 0.2,     //Roller in 
      xboxAux.getLeftTriggerAxis() >= 0.2,      //Roller out
      xboxAux.getLeftStickButtonPressed(),      //Enter all-manual mode
     // xboxAux.getStartButtonPressed(),     commented due 2 conflict 4/4 
      false,      //Soft limit override
      commandedState
    );

    commandedState = COMMAND_STATE_DO_NOTHING;

  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  disabledXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    currentTime.stop();
    
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic()
  {

  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  testXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  simulateXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  Misc
  *
  *----------------------------------------------------------------------------------------*/  
  public void determineCommandState(int xboxGamePieceSelection,
                                    boolean xboxLowNode,
                                    boolean xboxMidNode,
                                    boolean HighNode,
                                    boolean xboxStowPos,
                                    boolean xboxPickUpPos) 
  {
    selectedGamePiece = xboxGamePieceSelection;
    
    if(xboxStowPos)
    {
      commandedState = COMMAND_STATE_STOW;
    }
    else if (xboxLowNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedState = COMMAND_STATE_SCORE_LOW_CUBE;
      }
      else
      {
        commandedState = COMMAND_STATE_SCORE_LOW_CONE;
      }
    }
    else if(xboxMidNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedState = COMMAND_STATE_SCORE_MID_CUBE;
      }
      else
      {
        commandedState = COMMAND_STATE_SCORE_MID_CONE;
      }
    }
    else if(HighNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedState = COMMAND_STATE_SCORE_HIGH_CUBE;
      }
      else
      {
        commandedState = COMMAND_STATE_SCORE_HIGH_CONE;
      }
    }
    else if(xboxPickUpPos)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedState = COMMAND_STATE_PICKUP_CUBE;
      }
      else
      {
        commandedState = COMMAND_STATE_PICKUP_CONE;
      }
    }
    else{
      commandedState = COMMAND_STATE_DO_NOTHING;
    }

    SmartDashboard.putNumber("state", commandedState);
    SmartDashboard.putNumber("game piece", selectedGamePiece);
  }   //end of determineCommandState()
  
}