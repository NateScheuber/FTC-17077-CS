package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public static int home = 0;
    public static int level1 = 300;
    public static int level2 = 600;
    public static int level3 = 900;

    //claw items
    public static double closed = 0.2;
    public static double open = 0.5;
    public boolean clawClosed = false;
    public static double yesBlock = 0.3;
    public static double noBlock = 0.1;
    public static double flipIn = 0;
    public static double flipOut = 0.5;


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
    private CRServoImplEx clawFlipA   = null;
    private CRServoImplEx clawFlipB   = null;
    private ServoImplEx launchRelease = null;
    private ServoImplEx climbReleaseA = null;
    private ServoImplEx climbReleaseB = null;
    private ServoImplEx intakeLink = null;
    private ServoImplEx pixelBlock = null;
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
        liftMaster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSlave.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMaster.setPositionPIDFCoefficients(liftP);
        liftSlave.setPositionPIDFCoefficients(liftP);
        liftMaster.setTargetPositionTolerance(liftPT);
        liftSlave.setTargetPositionTolerance(liftPT);


        intakeMA = myOpMode.hardwareMap.get(DcMotorEx.class, "intakeMA");
        intakeMB = myOpMode.hardwareMap.get(DcMotorEx.class, "intakeMB");
        intakeMB.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = myOpMode.hardwareMap.get(ServoImplEx.class, "claw");
        clawFlipA = myOpMode.hardwareMap.get(CRServoImplEx.class, "clawFlipA");
        clawFlipB = myOpMode.hardwareMap.get(CRServoImplEx.class, "clawFlipB");
        climbReleaseA = myOpMode.hardwareMap.get(ServoImplEx.class, "climbReleaseA");
        climbReleaseB = myOpMode.hardwareMap.get(ServoImplEx.class, "climbReleaseB");
        launchRelease = myOpMode.hardwareMap.get(ServoImplEx.class, "launchRelease");
        pixelBlock = myOpMode.hardwareMap.get(ServoImplEx.class, "pixelBlock");

        intakeSA = myOpMode.hardwareMap.get(CRServoImplEx.class, "intakeSA");
        intakeSB = myOpMode.hardwareMap.get(CRServoImplEx.class, "intakeSB");
        intakeSB.setDirection(DcMotorSimple.Direction.REVERSE);

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawFlipA.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawFlipB.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSA.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSB.setPwmRange(new PwmControl.PwmRange(500, 2500));

        launchRelease.setPosition(0);
        climbReleaseA.setPosition(0);
        climbReleaseB.setPosition(1);
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

    //intake stuff
    public void intake(double speed){
        intakeMA.setPower(speed);
        intakeMB.setPower(speed);
        intakeSA.setPower(speed);
        intakeSB.setPower(speed);
    }

    //lift stuff
    public void lift(int position){
        liftMaster.setTargetPosition(position);
        liftSlave.setTargetPosition(-position);
        liftMaster.setPower(liftS);
        liftSlave.setPower(liftS);
    }
    public int liftTargetPosition(){ return liftMaster.getTargetPosition();}
    public int liftCurrentPosition(){return liftMaster.getCurrentPosition();}
    public boolean liftBusy(){return liftMaster.isBusy();}

    //claw stuff
    public void closeClaw(boolean toggle){
        if(toggle){
            claw.setPosition(closed);
            clawClosed = true;
        }
        else{
            claw.setPosition(open);
            clawClosed = false;
        }
    }
    public boolean isClawClosed(){ return clawClosed; }

    public void pixelBlocker(){
        if(clawClosed){
            pixelBlock.setPosition(noBlock);
        }
        else if(liftCurrentPosition()>50){
            pixelBlock.setPosition(noBlock);
        }
        else if(liftCurrentPosition()<50){
            pixelBlock.setPosition(yesBlock);
        }
    }

    public void clawFlip(){
        if(liftCurrentPosition()>200){
            clawFlipA.setPower(flipOut);
            clawFlipB.setPower(1-flipOut);
        }
        else{
            clawFlipA.setPower(flipIn);
            clawFlipB.setPower(1-flipIn);
        }
    }

    //endgame
    public void launch(){
        launchRelease.setPosition(0.5);
    }

    public void climb(){
        climbReleaseA.setPosition(0.25);
        climbReleaseB.setPosition(0.75);
    }
}
