// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;

import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzElevator;
import frc.Mechanisms.CatzArm;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzRGB;
import frc.Mechanisms.ColorMethod;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
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

  public double  xboxGamePieceSelection = 0.0;
  public double  xboxElevatorManualPwr    = 0.1;
  public boolean xboxElevatorManualMode = false;
  public boolean xboxStowPos   = false;
  public boolean xboxLowNode   = false;
  public boolean xboxMidNode   = false;
  public boolean xboxHighNode  = false;
  public boolean xboxPickUpGroundPos = false;
  public boolean xboxPickUpSinglePos = false;
  public boolean xboxPickUpDoublePos = false;

  public static boolean xboxNULL      = false;


  public static final int COMMAND_STATE_NULL            =  0;

  public static final int COMMAND_UPDATE_PICKUP_GROUND_CONE     = 1;
  public static final int COMMAND_UPDATE_PICKUP_GROUND_CUBE     = 2;
  public static final int COMMAND_UPDATE_PICKUP_SINGLE_CONE     = 3;
  public static final int COMMAND_UPDATE_PICKUP_SINGLE_CUBE     = 4;
  public static final int COMMAND_UPDATE_PICKUP_DOUBLE_CONE     = 5;
  public static final int COMMAND_UPDATE_PICKUP_DOUBLE_CUBE     = 6;

  public static final int COMMAND_UPDATE_STOW            = 7;

  public static final int COMMAND_UPDATE_SCORE_LOW_CONE  = 10;
  public static final int COMMAND_UPDATE_SCORE_LOW_CUBE  = 11;
  public static final int COMMAND_UPDATE_SCORE_MID_CONE  = 12;
  public static final int COMMAND_UPDATE_SCORE_MID_CUBE  = 13;
  public static final int COMMAND_UPDATE_SCORE_HIGH_CONE = 14;
  public static final int COMMAND_UPDATE_SCORE_HIGH_CUBE = 15;

  public static int commandedStateUpdate = COMMAND_STATE_NULL;

  public final static int GP_NULL = 0;
  public final static int GP_CUBE = 1;
  public final static int GP_CONE = 2;

  public static int selectedGamePiece = GP_CUBE;

  public static final int MODE_AUTO        = 0;
  public static final int MODE_MANUAL_HOLD = 1;
  public static final int MODE_MANUAL      = 2;
    
  

  private final double OFFSET_DELAY = 0.5;    //TBD put into AUTO BALANCE class

  //---------------------------------------------------------------------------------------------
  //  Mechanisms
  //---------------------------------------------------------------------------------------------
  public static CatzDrivetrain drivetrain;
  public static CatzElevator   elevator;
  public static CatzArm        arm;
  public static CatzIntake     intake;
  public static CatzRGB        led = new CatzRGB();


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


    //----------------------------------------------------------------------------------------------
    //  Mechanisms
    //----------------------------------------------------------------------------------------------
    drivetrain = new CatzDrivetrain();
    elevator   = new CatzElevator();
    arm        = new CatzArm();
    intake     = new CatzIntake();

  }

  public enum mechMode
  {
    AutoMode(Color.kGreen),
    ManualHoldMode(Color.kCyan),
    ManualMode(Color.kRed);

    public Color color;
    mechMode(Color color){
      this.color = color;
    }
  }

  public enum gamePiece{
    Cube(Color.kPurple),
    Cone(Color.kYellow),
    None(Color.kGhostWhite);

    public Color color;
    gamePiece(Color color){
      this.color = color;
    }
  }

  public enum gameModeLED{
    Autobalancing(led.oneColorFill, Color.kGreen),
    InAutonomous(led.startFlowing, led.PHANTOM_SAPPHIRE, Color.kWhite),
    MatchEnd(led.startFlowingRainbow),
    EndgameWheelLock(led.oneColorFillAllianceColor), 
    TeleOp(led.doNothing);

    public ColorMethod method;
    public Color[] color;
    private gameModeLED(ColorMethod method, Color... color)
    {
      this.method = method;
      this.color = color;
    }
  }

  public static mechMode intakeControlMode = mechMode.AutoMode;
  public static mechMode elevatorControlMode = mechMode.AutoMode;
  public static mechMode armControlMode = mechMode.AutoMode;
  public static gameModeLED currentGameModeLED = gameModeLED.MatchEnd;
  public static gamePiece currentGamePiece = gamePiece.None;

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
    arm.checkLimitSwitches();
    dataCollection.updateLogDataID(); 
    led.LEDPeriodic();

    //----------------------------------------------------------------------------------------------
    //  Shuffleboard Data Display
    //----------------------------------------------------------------------------------------------
    SmartDashboard.putNumber("NavX", navX.getAngle());
    SmartDashboard.putNumber("gamepiece int", selectedGamePiece);
    SmartDashboard.putNumber("COMMAND STATE", commandedStateUpdate);

    drivetrain.smartDashboardDriveTrain();
    drivetrain.smartDashboardDriveTrain_DEBUG();
    elevator.smartDashboardElevator();
    elevator.smartDashboardElevator_DEBUG();

        
    arm.smartDashboardARM();
  
    //debug should be commented out for comp

        intake.smartdashboardIntakeDebug();
       
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
    drivetrain.setBrakeMode();
    currentTime.reset();
    currentTime.start();

    navX.reset();

    navX.setAngleAdjustment(-navX.getYaw() + 180.0); //set navx's zero position to opposite way robot is facing
    
    currentGameModeLED = gameModeLED.InAutonomous;

    Timer.delay(OFFSET_DELAY);  //TBD - This should be 

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
    intake.enablePID(false);
    currentTime.reset();
    currentTime.start();

    dataCollection.startDataCollection();
    currentGameModeLED = gameModeLED.TeleOp;
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
  {

    drivetrain.cmdProcSwerve(xboxDrv.getLeftX(), xboxDrv.getLeftY(), xboxDrv.getRightX(), navX.getAngle(), xboxDrv.getRightTriggerAxis());

     if(xboxDrv.getStartButtonPressed())
     {
       zeroGyro();
     }



    xboxHighNode           = xboxAux.getYButton();
    xboxMidNode            = xboxAux.getBButton();
    xboxLowNode            = xboxAux.getAButton();
    xboxStowPos            = xboxAux.getXButton()     | xboxDrv.getRightStickButton();
    xboxPickUpGroundPos    = xboxAux.getStartButton() | xboxDrv.getLeftStickButton();

    xboxElevatorManualMode = xboxAux.getRightStickButton();
    xboxElevatorManualPwr  = xboxAux.getRightY();
    boolean xboxRollerOutEnabled = true;

    //Bumper overiding flinging
    if(xboxAux.getPOV() != DPAD_UP)
    {
      xboxRollerOutEnabled = xboxAux.getLeftBumper();
    }

 
    xboxGamePieceSelection(xboxAux.getPOV(),                // Left = Cone, Right = Cube
                            xboxAux.getBackButtonPressed()); // Clear Selected Game Piece

    determineCommandState(xboxGamePieceSelection, xboxLowNode, 
                                                   xboxMidNode, 
                                                   xboxHighNode, 
                                                   xboxStowPos,
                                                   xboxPickUpGroundPos,
                                                   xboxAux.getPOV() == DPAD_DN,
                                                   false);
  
                                                   
    elevator.cmdProcElevator(xboxElevatorManualPwr,  // Manual and Manual Hold Elevator Power
                            xboxElevatorManualMode,  // Enter Manual Mode
                            commandedStateUpdate);
                            
                            

    arm.cmdProcArm(xboxAux.getRightTriggerAxis() >= 0.1,   //Manual Extend Arm
                   xboxAux.getLeftTriggerAxis()  >= 0.1,   //Manual Retract Arm 
                   commandedStateUpdate); 
                   

    intake.cmdProcIntake(-xboxAux.getLeftY(),                   //Semi-manual override
                          xboxAux.getRightBumper(),             //Roller in 
                          xboxRollerOutEnabled,              //Roller out
                          xboxAux.getLeftStickButtonPressed(),  //Enter all-manual mode
                          (xboxAux.getRightBumper() & xboxAux.getLeftBumper()),  //Soft limit override
                          commandedStateUpdate,
                          selectedGamePiece);


                         
    commandedStateUpdate = COMMAND_STATE_NULL;



    
    // Lock Wheels (Balancing)
     if(xboxDrv.getBButton())
     {
       drivetrain.lockWheels();
     }

     /*
     if(DriverStation.getMatchTime() < 2.0)
     {
       led.matchDone = true;

     }
     else if(DriverStation.getMatchTime() < 15.0)
     {
       led.endGame = true;
     }
     */
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

   System.out.println( "intake temp " + intake.intakeWristTemp());
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
  public void testInit() {
    drivetrain.setWheelOffsets(drivetrain.getOffsetAverages());
  }

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
  public void determineCommandState(double  xboxGamePieceSelection,
                                    boolean LowNode,
                                    boolean MidNode,
                                    boolean HighNode,
                                    boolean StowPos,
                                    boolean PickUpGroundPos,
                                    boolean PickUpSinglePos,
                                    boolean PickUpDoublePos) 
  {
    if(StowPos)
    {
      commandedStateUpdate = COMMAND_UPDATE_STOW;
    }
    else if (LowNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_LOW_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_LOW_CONE;
      }
    }
    else if(MidNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_MID_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_MID_CONE;
      }
    }
    else if(HighNode)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_HIGH_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_SCORE_HIGH_CONE;
      }
    }
    else if(PickUpGroundPos)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_GROUND_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_GROUND_CONE;
      }
    }
    else if(PickUpSinglePos)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_SINGLE_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_SINGLE_CONE;
      }

    }
    else if(PickUpDoublePos)
    {
      if(selectedGamePiece == GP_CUBE)
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_DOUBLE_CUBE;
      }
      else
      {
        commandedStateUpdate = COMMAND_UPDATE_PICKUP_DOUBLE_CONE;
      }
    }
  }   //end of determineCommandState()



  public void zeroGyro()
  {
    navX.setAngleAdjustment(-navX.getYaw());
  }

  public void xboxGamePieceSelection(double Dpad, boolean SelectButtonPressed)
  {
    if(Dpad == DPAD_LT)
    {
      selectedGamePiece = GP_CONE;
      currentGamePiece = gamePiece.Cone;
    }
    else if(Dpad == DPAD_RT)
    {
      selectedGamePiece = GP_CUBE;
      currentGamePiece = gamePiece.Cube;
    }
    else if(SelectButtonPressed)
    {
      selectedGamePiece = GP_NULL;
      currentGamePiece = gamePiece.None;
    }
  }

}