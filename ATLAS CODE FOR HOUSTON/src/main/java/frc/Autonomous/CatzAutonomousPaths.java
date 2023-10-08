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
    private final double BIAS_OFFSET =   0.0;
    private final double FWD_OR_BWD  =   0.0 + BIAS_OFFSET;
    private final double RIGHT       =   90.0; 
    private final double LEFT        =   -90.0;

    private final double INDEXER_EJECT_TIME = 0.5;  //TBD - Put in Indexer

    /*------------------------------------------------------------------------------------
    *  Path ID's
    *-----------------------------------------------------------------------------------*/
    private final int LEFT_SCORE_1                        = 1;
    private final int LEFT_SCORE_2                        = 2;
    private final int LEFT_SCORE_1_BALANCE                = 3;

    private final int CENTER_SCORE_1_LOW                  = 20;
    private final int CENTER_SCORE_1_MID_BALANCE          = 21;
    private final int CENTER_SCORE_1_HIGH_BALANCE         = 22;

    private final int RIGHT_SCORE_1                       = 40;
    private final int RIGHT_SCORE_2                       = 41;
    private final int RIGHT_SCORE_1_BALANCE               = 42;

    private final int RIGHT_SCORE_3                       = 50;
    private final int LEFT_SCORE_3                        = 51;

    private final int TEST                            = 100;



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

        chosenPath.addOption       ("Center Score 1 Low",          CENTER_SCORE_1_LOW);
        chosenPath.addOption       ("Center Score 1 Mid Balance",  CENTER_SCORE_1_MID_BALANCE);
        chosenPath.addOption       ("Center Score 1 High Balance", CENTER_SCORE_1_HIGH_BALANCE);

        chosenPath.addOption       ("Right Score 1",          RIGHT_SCORE_1);
        chosenPath.addOption       ("Right Score 2",          RIGHT_SCORE_2);
        chosenPath.addOption       ("Right Score 1 Balance",  RIGHT_SCORE_1_BALANCE);

        chosenPath.addOption       ("Right Score 3",          RIGHT_SCORE_3);
        chosenPath.addOption       ("Left Score 3",           LEFT_SCORE_3);

        chosenPath.addOption       ("TEST PATH",  TEST);

        SmartDashboard.putData     ("Auton Path", chosenPath);
    
    }


    public void executeSelectedPath()
    {
        pathID = chosenPath.getSelected();

        System.out.println("PathID: " + pathID);

        switch (pathID)
        {
            case LEFT_SCORE_1: sideScore1(); //Scores High Cone - TBD
            break;

            case LEFT_SCORE_2: LeftScore2(); //Scores High Cone + Low Cone - TBD
            break;

            case LEFT_SCORE_1_BALANCE: LeftScore1Balance(); //Scores High Cone - TBD
            break;

            case CENTER_SCORE_1_LOW: centerScore1HighCone(); 
            break;

            case CENTER_SCORE_1_MID_BALANCE: centerScore1MidBalance();  //Scores Mid Cone - TBD
            break;

            case CENTER_SCORE_1_HIGH_BALANCE: centerScore1HighConeBalance(); //Scores High Cone - TBD
            break;

            case RIGHT_SCORE_1: sideScore1(); //Scores High Cone - TBD
            break;

            case RIGHT_SCORE_2: RightScore2(); //Scores High Cone + Low Cone - TBD
            break;

            case RIGHT_SCORE_1_BALANCE: RightScore1Balance(); //Scores High Cone - TBD
            break;

            case RIGHT_SCORE_3: RightScore3(); //scoring 3 mid cone 2 low cube
            break;

            case TEST: testPath(); //Scores High Cone - TBD
            break;
        }

    }


    public void testPath()
    {
        Robot.auton.TurnInPlace(-180 , 2.0); 
        Robot.auton.DriveStraight(-20, FWD_OR_BWD, 2.0);

    }



    /*-----------------------------------------------------------------------------------------
    *    
    *  Auton Functions
    * 
    *----------------------------------------------------------------------------------------*/
    public void Balance()
    {
        Robot.balance.StartBalancing();
    }


    /*-----------------------------------------------------------------------------------------
    *    
    *  Auton Paths
    * 
    *----------------------------------------------------------------------------------------*/
    public void sideScore1()
    {
        scoreConeHigh();

        Robot.auton.DriveStraight(200, FWD_OR_BWD,  4.0);     //From Grid to exit community
    }

    public void sideScore1Pickup1SingleSubSide()
    {
        double offset = 5;
        double flipConstant;
        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE) //angle offset depending on what side is red alliance or blue alliance so it can pick up cubes
        {
            flipConstant = 1;
        }
        else
        {
            flipConstant = -1;
        }
        scoreConeHigh();

        Robot.auton.DriveStraight(30, FWD_OR_BWD, 1.0);     //From Grid to area to do 180 deg turn //TBD remove drv straight 30 and add to the second drive straight

        Robot.auton.TurnInPlace(flipConstant*(180 + offset), 1.0);

        pickUpCube();
        
        Robot.auton.DriveStraight(196, FWD_OR_BWD, 2.5);  //191
        Timer.delay(0.4); 
        stow();

        Robot.auton.TurnInPlace(flipConstant*(180 + -offset), 1.0);
    }

    public void sideScore1Pickup1NoSingleSubside()
    {

        double offset = 5;
        double flipConstant;
        scoreConeHigh();
        double direction;
        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = RIGHT;
            flipConstant = -1;

        }
        else
        {
            direction = LEFT;
            flipConstant = 1;
        }
        
        Robot.auton.DriveStraight(-27, direction, 1.0);     //From Grid to area to do 180 deg turn  //TBD remov initla drvstr

        Robot.auton.TurnInPlace(flipConstant*(180 + -offset) , 1.0);

        pickUpCube();
        
        Robot.auton.DriveStraight(224, FWD_OR_BWD, 3.0);  //191
        stow();

        Robot.auton.TurnInPlace(flipConstant*(180 + offset), 1.0); 
    }


    
    public void LeftScore2()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = RIGHT;
            sideScore1Pickup1NoSingleSubside();
        }
        else
        {
            direction = LEFT;
            sideScore1Pickup1SingleSubSide();
        }

        Robot.auton.DriveStraight(-220,  FWD_OR_BWD, 3.0);    //Move in front of center node

        
        Robot.auton.DriveStraight(40,  direction, 1.0);    //Move in front of center node

        scoreCubeHigh();
        Timer.delay(0.3);
        stow();

        Robot.auton.DriveStraight(-40,  direction, 2.0); 

        Robot.auton.DriveStraight(175, FWD_OR_BWD, 5.0);
    }

        
    public void RightScore2()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = LEFT;
            sideScore1Pickup1SingleSubSide();
        }
        else
        {
            direction = RIGHT;
            sideScore1Pickup1NoSingleSubside();
        }


        Robot.auton.DriveStraight(-220, FWD_OR_BWD, 3.0);

        Robot.auton.DriveStraight( 40,  direction, 1.0);    //Move in front of center node

        scoreCubeHigh();
        Timer.delay(0.3);
        stow();

        Robot.auton.DriveStraight(-40,  direction, 1.0); 

        Robot.auton.DriveStraight(175, FWD_OR_BWD, 3.0);
    }

    public void RightScore3()
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

        sideScore1Pickup1NoSingleSubside();
        
        Robot.auton.DriveStraight(-90, FWD_OR_BWD, 3.0);

        setCommandStateAutonIntakeDelay(Robot.COMMAND_UPDATE_SCORE_MID_CONE, Robot.GP_CUBE); //fling cube
        Timer.delay(0.2); 
        stow();

        Robot.auton.DriveStraight(110, FWD_OR_BWD, 3.0);
                                   
        Robot.auton.DriveStraight(80,  direction, 2.0);    //Move in behind cube
        pickUpCube();    
        Robot.auton.DriveStraight(-100, FWD_OR_BWD, 3.0);
        stow();
        Timer.delay(0.2);
        setCommandStateAutonIntakeDelay(Robot.COMMAND_UPDATE_SCORE_MID_CUBE, Robot.GP_CUBE);

    
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

        sideScore1Pickup1SingleSubSide();

        Robot.auton.DriveStraight(-42, FWD_OR_BWD, 5.0);
        Robot.auton.DriveStraight(80, direction, 2.0); //prev 70
        Robot.auton.DriveStraight(-80, FWD_OR_BWD, 2.0);
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

        sideScore1Pickup1SingleSubSide();

        Robot.auton.DriveStraight(-42, FWD_OR_BWD, 5.0);
        Robot.auton.DriveStraight(80, direction, 2.0); //prev 70
        Robot.auton.DriveStraight(-80, FWD_OR_BWD, 2.0);
        Balance();


    }

    public void centerScore1MidBalance() 
    {
        centerScore1MidCone(); 
        Robot.auton.DriveStraight(-70, FWD_OR_BWD, 4.0); 
        Balance();
    }

    public void centerScore1HighConeBalance() 
    {
        centerScore1HighCone();
        Timer.delay(0.5);//wait for balance to level out
        Robot.auton.DriveStraight(-115, FWD_OR_BWD, 2.0);
        Balance();
    }
 
    public void centerScore1HighCone() 
    {
        scoreConeHigh();
        Robot.auton.DriveStraightoffChargeStation(170, FWD_OR_BWD, 4.0);
    }

    public void centerScore1MidCone() 
    {
        scoreConeMid();
        Robot.auton.DriveStraight(176, FWD_OR_BWD, 4.0); //TBD update timeout and the distance cnts
    }




    public void scoreConeLow()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_LOW_CONE, Robot.GP_CONE);

        scoreCone();
    }

    public void scoreConeMid()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_MID_CONE, Robot.GP_CONE);
        Timer.delay(0.4);
        scoreCone();
    }

    public void scoreConeHigh()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_HIGH_CONE, Robot.GP_CONE);
        Timer.delay(0.5);
        scoreCone();
    }


    public void scoreCubeLow()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_LOW_CUBE, Robot.GP_CUBE);

        scoreCube();
    }

    public void scoreCubeMid()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_MID_CUBE, Robot.GP_CUBE);

        scoreCube();
    }

    public void scoreCubeHigh()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE, Robot.GP_CUBE);

        scoreCube();
    }

    public void scoreCube()
    {
        ejectCube();
        setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
        Robot.intake.rollersOff();
    }

    public void scoreCone()
    {
        ejectCone();
        setCommandStateAuton(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
        Robot.intake.rollersOff();
    }

    public void setScorePosCube()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_SCORE_LOW_CUBE, Robot.GP_CUBE);
    }

    public void ejectCube()
    {
        Robot.intake.rollersOutCube();
        Timer.delay(0.1); //TBD will need to change
    }

    public void ejectCone()
    {
        Robot.intake.rollersOutCone();
        Timer.delay(0.2);
        
    }

    public void setCommandStateAuton(int cmdState, int gamePiece)
    {
        int timeout = 0;
        boolean done = false;

        Robot.selectedGamePiece = gamePiece;

        Robot.elevator.cmdProcElevator(0.0,   false, cmdState);
        Robot.arm.cmdProcArm          (false, false, cmdState);
        Robot.intake.cmdProcIntake    (0.0, false, false, false, false, cmdState, gamePiece);

        if(cmdState != Robot.COMMAND_UPDATE_STOW)
        {
            while(!done)
            {
                if(Robot.elevator.isElevatorInPos() == true && 
                   Robot.arm.isArmInPos()           == true && 
                   Robot.intake.isIntakeInPos()     == true)
                {
                    done = true;
                }

                timeout++;
                if(timeout >= 80)//prev 200
                {
                    done = true;
                }

                Timer.delay(0.010);
            }
        }
    }

    
    public void setCommandStateAutonFast(int cmdState, int gamePiece)
    {
        int timeout = 0;
        boolean done = false;

        Robot.selectedGamePiece = gamePiece;

        Robot.elevator.cmdProcElevator(0.0,   false, cmdState);
        Robot.arm.cmdProcArm          (false, false, cmdState);
        Robot.intake.cmdProcIntake    (0.0, false, false, false, false, cmdState, gamePiece);
    }

    public void setCommandStateAutonIntakeDelay(int cmdState, int gamePiece)
    {
        int timeout = 0;
        boolean done = false;

        Robot.selectedGamePiece = gamePiece;

        Robot.elevator.cmdProcElevator(0.0,   false, Robot.COMMAND_UPDATE_SCORE_MID_CONE);
        Robot.arm.cmdProcArm          (false, false, Robot.COMMAND_UPDATE_SCORE_HIGH_CONE);

        if(cmdState != Robot.COMMAND_UPDATE_STOW)
        {
            Timer.delay(0.5);
            Robot.intake.cmdProcIntake    (0.0, false, false, false, false, Robot.COMMAND_UPDATE_SCORE_HIGH_CONE, gamePiece);
            ejectCube();

        }
    }

    public void setCommandStateAutonIntake(int cmdState, int gamePiece)
    {

        Robot.selectedGamePiece = gamePiece;

        Robot.elevator.cmdProcElevator(0.0,   false, Robot.COMMAND_UPDATE_SCORE_MID_CONE);
        Robot.arm.cmdProcArm          (false, false, Robot.COMMAND_UPDATE_SCORE_HIGH_CONE);

        if(cmdState != Robot.COMMAND_UPDATE_STOW)
        {
            Timer.delay(0.5);
            Robot.intake.cmdProcIntake    (0.0, false, false, false, false, Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE, gamePiece);
            ejectCube();

        }
    }

    public void pickUpCone()
    {
        setCommandStateAuton(Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE, Robot.GP_CONE);
        Robot.intake.rollersInCone();
    }

    public void pickUpCube()
    {
        setCommandStateAutonFast(Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE, Robot.GP_CUBE);
        Robot.intake.rollersInCube();
    }

    public void stow()
    {
        Robot.intake.rollersOff();
        setCommandStateAutonFast(Robot.COMMAND_UPDATE_STOW, Robot.GP_NULL);
    }


}