package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.List;

public class berthaHardware {
    private LinearOpMode myOpMode = null;


    //lift items
    public static double liftP = 0;
    public static double liftS = 0.5;
    public static int liftPT = 20;

    public static int level1 = 300;
    public static int level2 = 600;
    public static int level3 = 900;

    public enum liftPosition{
        home,
        low,
        medium,
        high
    }
    liftPosition currentHeight = liftPosition.home;

    //motors
    private DcMotorEx frontRight    = null;
    private DcMotorEx frontLeft     = null;
    private DcMotorEx backRight     = null;
    private DcMotorEx backLeft      = null;
    private DcMotorEx liftMaster    = null;
    private DcMotorEx liftSlave     = null;
    private DcMotorEx intakeMA      = null;
    private DcMotorEx intakeMB      = null;

    //servos
    private ServoImplEx claw        = null;
    private ServoImplEx clawFlipA   = null;
    private ServoImplEx clawFlipB   = null;
    private ServoImplEx launchRelease = null;
    private ServoImplEx climbReleaseA = null;
    private ServoImplEx climbReleaseB = null;
    private ServoImplEx intakeLink = null;
    private ServoImplEx clawBlock = null;
    private CRServoImplEx intakeSA  = null;
    private CRServoImplEx intakeSB  = null;

    public berthaHardware(LinearOpMode opmode){myOpMode = opmode;}

    public void init(){
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        liftMaster = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMaster");
        liftSlave = myOpMode.hardwareMap.get(DcMotorEx.class, "liftSlave");

        intakeMA = myOpMode.hardwareMap.get(DcMotorEx.class, "intakeMA");
        intakeMB = myOpMode.hardwareMap.get(DcMotorEx.class, "intakeMB");
        intakeMB.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = myOpMode.hardwareMap.get(ServoImplEx.class, "claw");
        clawFlipA = myOpMode.hardwareMap.get(ServoImplEx.class, "clawFlipA");
        clawFlipB = myOpMode.hardwareMap.get(ServoImplEx.class, "clawFlipB");
        climbReleaseA = myOpMode.hardwareMap.get(ServoImplEx.class, "climbReleaseA");
        climbReleaseB = myOpMode.hardwareMap.get(ServoImplEx.class, "climbReleaseB");
        launchRelease = myOpMode.hardwareMap.get(ServoImplEx.class, "launchRelease");

        intakeSA = myOpMode.hardwareMap.get(CRServoImplEx.class, "intakeSA");
        intakeSB = myOpMode.hardwareMap.get(CRServoImplEx.class, "intakeSB");
        intakeSB.setDirection(DcMotorSimple.Direction.REVERSE);

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawFlipA.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawFlipB.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSA.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSB.setPwmRange(new PwmControl.PwmRange(500, 2500));


    }
    public void driveRobotOriented(double X, double Y, double R){
        frontRight.setPower(Y-X-R);
        backRight.setPower(Y+X-R);
        frontLeft.setPower(Y+X+R);
        backLeft.setPower(Y-X+R);
    }
    public void driveSlowMo(boolean forward, boolean backward, boolean right, boolean left){
        if(forward){
            driveRobotOriented(0, 0.25, 0);
        }
        else if(backward){
            driveRobotOriented(0,-0.25,0);
        }

        if(right){
            driveRobotOriented(0.25, 0, 0);
        }
        else if(left){
            driveRobotOriented(-0.25, 0, 0);
        }
    }

    public void intake(double speed){
        intakeMA.setPower(speed);
        intakeMB.setPower(speed);
        intakeSA.setPower(speed);
        intakeSB.setPower(speed);
    }

    public int liftTargetPosition(){
        return liftMaster.getTargetPosition();
    }
    public int liftCurrentPosition(){
        return liftMaster.getCurrentPosition();
    }
    public boolean liftBusy(){return liftMaster.isBusy();}
}
