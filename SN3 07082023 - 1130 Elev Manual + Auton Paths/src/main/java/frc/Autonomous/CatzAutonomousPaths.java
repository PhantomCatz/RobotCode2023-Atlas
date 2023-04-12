package frc.Autonomous;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

/*****************************************************************************************
*
* Autonomous selections
* 
*****************************************************************************************/
@SuppressWarnings("unused")
public class CatzAutonomousPaths
{  
    public final SendableChooser<Boolean> chosenAllianceColor = new SendableChooser<>();
    private final SendableChooser<Integer> chosenPath         = new SendableChooser<>();

    /*------------------------------------------------------------------------------------
    *  Field Relative angles when robot is TBD - Finish Comment  
    *-----------------------------------------------------------------------------------*/
    private final double FWD_OR_BWD  =   0.0;
    private final double RIGHT       =  -90.0; 
    private final double LEFT        =   90.0;
    private final double BIAS_OFFSET =  -2.0;

    private final double INDEXER_EJECT_TIME = 0.5;  //TBD - Put in Indexer

    /*------------------------------------------------------------------------------------
    *  Path ID's
    *-----------------------------------------------------------------------------------*/
    private final int LEFT_SCORE_1                        = 1;
    private final int LEFT_SCORE_2                        = 2;
    private final int LEFT_SCORE_1_BALANCE                = 3;

    private final int CENTER_SCORE_1_LOW                    = 20;
    private final int CENTER_SCORE_1_MID_BALANCE            = 21;
    private final int CENTER_SCORE_1_HIGH_BALANCE           = 22;

    private final int RIGHT_SCORE_1                        = 40;
    private final int RIGHT_SCORE_2                        = 41;
    private final int RIGHT_SCORE_1_BALANCE                = 42;

    private final int TEST                  = 100;



    public static int pathID;

    public static int autoState = Robot.COMMAND_STATE_NULL;

    public static Thread autoThread;

     /*  DRIVE STRAIGHT VALUES: 
     * if distance > 70, then FAST, else SLOW
     * 8 second maxTime is an arbitrary number, subject to change upon more testing 
     * only robot backwards movement has negative signs over distance and maxspeed
     * left and right distances and max speed aren't negative
     * TEMP_DECEL_DIST decelDistance is an arbitrary number, subject to change upon more testing
     * 
     *   *note* - autonomous is 15 seconds, meaning that all of this will have to finsih within that time
     *          - THIS CODE IS MADE FOR BLUE SIDE 
     *          - FOR RED, CHANGE LEFTS WITH RIGHTS AND RIGHTS WITH LEFTS (from blue)
     *          - movement similar to code.org level programming
    */

    /* PATH NAME:
     *    /CenterRightTunnel/
     * - CenterRight (Starting Position)
     * - Tunnel (type of movement/movement path)
     */

    /* Distances:          -______-
     * drive.DriveStraight(distance, decelDistance, maxSpeed, wheelPos, maxTime);
     *  - 224 = distance from grid to center pieces
     *                
     */
    // drive.DriveStraight(distance, decelDist, )


    public CatzAutonomousPaths()
    {
        chosenAllianceColor.setDefaultOption("Blue Alliance", Robot.constants.BLUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  Robot.constants.RED_ALLIANCE);
        SmartDashboard.putData              ("Alliance Color", chosenAllianceColor);

        chosenPath.setDefaultOption("Left Score 1",           LEFT_SCORE_1);   
        chosenPath.addOption       ("Left Score 2",           LEFT_SCORE_2);
        chosenPath.addOption       ("Left Score 1 Balance",   LEFT_SCORE_1_BALANCE);

        chosenPath.addOption       ("Center Score 1 Low",         CENTER_SCORE_1_LOW);
        chosenPath.addOption       ("Center Score 1 Mid Balance", CENTER_SCORE_1_MID_BALANCE);
        chosenPath.addOption       ("Center Score 1 High Balance", CENTER_SCORE_1_HIGH_BALANCE);

        chosenPath.addOption       ("Right Score 1",          RIGHT_SCORE_1);
        chosenPath.addOption       ("Right Score 2",          RIGHT_SCORE_2);
        chosenPath.addOption       ("Right Score 1 Balance",  RIGHT_SCORE_1_BALANCE);

        chosenPath.addOption       ("TEST PATH",  TEST);

        SmartDashboard.putData     ("Auton Path", chosenPath);

    
    }


    public void executeSelectedPath()
    {
        pathID = chosenPath.getSelected();

        System.out.println("PathID: " + pathID);

        switch (pathID)
        {
            case LEFT_SCORE_1: SideScore1(); //Scores High Cone - TBD
            break;

            case LEFT_SCORE_2: LeftScore2(); //Scores High Cone + Low Cone - TBD
            break;

            case LEFT_SCORE_1_BALANCE: LeftScore1Balance(); //Scores High Cone - TBD
            break;

            case CENTER_SCORE_1_LOW: CenterScore1(); //Scores Low Cone - TBD
            break;

            case CENTER_SCORE_1_MID_BALANCE: CenterScore1MidBalance();  //Scores Mid Cone - TBD
            break;

            case CENTER_SCORE_1_HIGH_BALANCE: CenterScore1HighBalance(); //Scores High Cone - TBD
            break;

            case RIGHT_SCORE_1: SideScore1(); //Scores High Cone - TBD
            break;

            case RIGHT_SCORE_2: RightScore2(); //Scores High Cone + Low Cone - TBD
            break;

            case RIGHT_SCORE_1_BALANCE: RightScore1Balance(); //Scores High Cone - TBD
            break;

            case TEST: testPath(); //Scores High Cone - TBD
            break;
        }

    }

    public void testPath()
    {
        //scoreConeHigh();
        pickUpCone();
        Timer.delay(1.0);
        Robot.intake.rollersOff();
        Timer.delay(0.2);
        stow();
    }




   



  /*-----------------------------------------------------------------------------------------
   *    
   * * Auton Functions
   * 
   *----------------------------------------------------------------------------------------*/

    public void Balance()
    {
        Robot.balance.StartBalancing();
    }

    /*-------------------------------------------------------------------------
     *
     * CODE FROM OVERTIME
     * 
     * ------------------------------------------------------------------------
     */
    public void SideScore1()
    {
        scoreConeHigh();

        Robot.auton.DriveStraight(100, LEFT, 2.0);     //From Grid to area to do 180 deg turn
        Timer.delay(2.0);
        Robot.auton.DriveStraight(100, RIGHT, 2.0);     //From Grid to area to do 180 deg turn

    }

    public void sideScore2()
    {
        scoreConeHigh();

        Robot.auton.DriveStraight( 30, FWD_OR_BWD, 2.0);     //From Grid to area to do 180 deg turn

        Robot.auton.TurnInPlace(180, 2);

        pickUpCone();
        
        Robot.auton.DriveStraight(176, FWD_OR_BWD, 5.0); //was 200 before, too far; over center line...was 172 before, too short, now 
        Timer.delay(1.50);
        stow();
        Timer.delay(0.75);

        Robot.auton.TurnInPlace(180, 2);

        Robot.auton.DriveStraight(-214, FWD_OR_BWD, 5.0);
        scoreConeLow();
    }


    
    public void LeftScore2()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = RIGHT;
        }
        else
        {
            direction = LEFT;
        }

        sideScore2();

        Robot.auton.DriveStraight( 48,  direction, 2.0);    //Move in front of center node
        Robot.auton.DriveStraight(-25, FWD_OR_BWD, 2.0);    //Move up to center node

        scoreConeLow();
    }

        
    public void RightScore2()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = LEFT;
        }
        else
        {
            direction = RIGHT;
        }

        sideScore2();

        Robot.auton.DriveStraight( 48,  direction, 2.0);    //Move in front of center node
        Robot.auton.DriveStraight(-25, FWD_OR_BWD, 2.0);    //Move up to center node

        scoreConeLow();
    }

    public void LeftScore1Balance()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = RIGHT;
        }
        else
        {
            direction = LEFT;
        }

        scoreConeHigh();

        Robot.auton.DriveStraight(180, FWD_OR_BWD+BIAS_OFFSET, 7.0);     //From Grid to area to do 180 deg turn

        Robot.auton.TurnInPlace(180, 2);

        pickUpCone();
        Timer.delay(0.75);
        Robot.auton.DriveStraight(22, FWD_OR_BWD+BIAS_OFFSET, 2.0);
        Timer.delay(0.5);
        stow();
        Timer.delay(0.75);
        Robot.auton.TurnInPlace(180, 2);
        Robot.auton.DriveStraight(-42, FWD_OR_BWD+BIAS_OFFSET, 5.0);
        Robot.auton.DriveStraight(60, direction, 2.0);
        Robot.auton.DriveStraight(-80, FWD_OR_BWD+BIAS_OFFSET, 2.0);
        Balance();
    }


    public void RightScore1Balance()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = LEFT;
        }
        else
        {
            direction = RIGHT;
        }

        scoreConeHigh();

        Robot.auton.DriveStraight(30, FWD_OR_BWD, 2.0);     //From Grid to area to do 180 deg turn

        Robot.auton.TurnInPlace(180, 2);

        Robot.auton.DriveStraight(150, FWD_OR_BWD, 5.0);
        pickUpCone();
        Robot.auton.DriveStraight(22, FWD_OR_BWD, 2.0);
        stow();
        Robot.auton.TurnInPlace(180, 2);
        Robot.auton.DriveStraight(-42, FWD_OR_BWD, 5.0);
        Robot.auton.DriveStraight(48, direction, 2.0);
        Robot.auton.DriveStraight(-80, FWD_OR_BWD, 2.0);
        Balance();

    }

    public void CenterScore1MidBalance() 
    {
        scoreConeMid();
        Robot.auton.DriveStraight(110, FWD_OR_BWD, 4.0); //offset was previously -7 applied to FW_OR_BACK
        Balance();
    }

    public void CenterScore1HighBalance() 
    {
        scoreConeHigh();
        Robot.auton.DriveStraight(110, FWD_OR_BWD, 4.0);
        Balance();
    }
 
    public void CenterScore1() 
    {
        scoreConeLow();
        Robot.auton.DriveStraight(200, FWD_OR_BWD, 4.0);
    }





    public void scoreConeLow()
    {
       setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_LOW_CONE, Robot.GP_CONE);

       ejectCone();

       setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }

    public void scoreConeMid()
    {
       setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_MID_CONE, Robot.GP_CONE);

       ejectCone();

       setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }

    public void scoreConeHigh()
    {
        System.out.println("score cone high");
       setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_HIGH_CONE, Robot.GP_CONE);

       ejectCone();

       setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }


    public void scoreCubeLow()
    {
       setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_LOW_CUBE, Robot.GP_CUBE);

       ejectCube();

       setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }

    public void scoreCubeMid()
    {
       setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_MID_CUBE, Robot.GP_CUBE);

       ejectCube();

       setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }

    public void scoreCubeHigh()
    {
       setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE, Robot.GP_CUBE);

       ejectCube();

       setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }



    public void setCommandStateAuton(int cmdState, int gamePiece)
    {
        int timeout = 0;
        boolean done = false;

        System.out.println("set cmdstate auton block");

        Robot.elevator.cmdProcElevator(0.0,   false, cmdState);
        Robot.arm.cmdProcArm          (false, false, cmdState);
        Robot.intake.cmdProcIntake    (0.0, false, false, false, false, cmdState, gamePiece);

        while(!done && timeout <= 3000 && cmdState != Robot.COMMAND_UPDATE_STOW)
        {
            done = (Robot.elevator.isElevatorInPos() == true && Robot.arm.isArmInPos() == true && Robot.intake.isIntakeInPos() == true);
            timeout++;
            Timer.delay(0.001);
        }
    }



    public void ejectCube()
    {
        Robot.intake.rollersOutCube();
        Timer.delay(0.2);
        Robot.intake.rollersOff();
    }

    public void ejectCone()
    {
        Robot.intake.rollersOutCone();
        Timer.delay(1.0);
        Robot.intake.rollersOff();
    }

    public void pickUpCone()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE, Robot.GP_CONE);
        Robot.intake.rollersInCone();
    }

    public void pickUpCube()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE, Robot.GP_CUBE);
        Robot.intake.rollersInCube();
    }

    public void stow()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
        Robot.intake.rollersOff();
    }
}