package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class berthaHardware {
    private LinearOpMode myOpMode = null;

    //lift items
    public static double liftP = 0.004;
    public static double liftS = 0.5;
    public static int liftPT = 20;
    public static double liftPower = 0;
    public static int liftPosition = 0;

    public static int home = -5;
    public static int level1 = 700;
    public static int level2 = 950;
    public static int level3 = 1450;

    //claw items
    public static double closed = 0.295;
    public static double open = 0.44;
    public boolean clawClosed = false;
    public static double flipIn = 0.105;
    public static double flipOut = 0.32;
    public static double rotatePostion = 0;
    public static double rotate0 = 0;
    public static double rotate30 = 0.2;
    public static double rotate90 = 0.4;
    public boolean rotateTrigger = false;

    //intake items
    public static double intakeDown = 0.98;
    public static double intakeUp = 0.65;

    //motors
    private DcMotorEx frontRight    = null;
    private DcMotorEx frontLeft     = null;
    private DcMotorEx backRight     = null;
    private DcMotorEx backLeft      = null;
    private DcMotorEx liftMaster    = null;
    private DcMotorEx liftSlave     = null;
    private DcMotorEx intakeMA      = null;
    private DcMotorEx climb         = null;

    //servos
    private ServoImplEx claw        = null;
    private ServoImplEx clawFlip   = null;
    private CRServoImplEx clawRotate   = null;
    private ServoImplEx launchRelease = null;
    private ServoImplEx intakeLink = null;
    private ServoImplEx pixelBlock = null;
    private CRServoImplEx intakeSA  = null;
    private CRServoImplEx intakeSB  = null;
    private ServoImplEx pixelRelease = null;

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
        liftSlave.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftSlave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMA = myOpMode.hardwareMap.get(DcMotorEx.class, "intakeMA");
        climb = myOpMode.hardwareMap.get(DcMotorEx.class, "climb");
        intakeLink = myOpMode.hardwareMap.get(ServoImplEx.class, "intakeLink");

        claw = myOpMode.hardwareMap.get(ServoImplEx.class, "claw");
        clawFlip = myOpMode.hardwareMap.get(ServoImplEx.class, "clawFlip");
        clawRotate = myOpMode.hardwareMap.get(CRServoImplEx.class, "clawRotate");
        launchRelease = myOpMode.hardwareMap.get(ServoImplEx.class, "launchRelease");
        pixelBlock = myOpMode.hardwareMap.get(ServoImplEx.class, "pixelBlock");
        pixelRelease =myOpMode.hardwareMap.get(ServoImplEx.class, "pixelRelease");


        intakeSA = myOpMode.hardwareMap.get(CRServoImplEx.class, "intakeSA");
        intakeSB = myOpMode.hardwareMap.get(CRServoImplEx.class, "intakeSB");
        intakeSA.setDirection(DcMotorSimple.Direction.REVERSE);

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawFlip.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawRotate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSA.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSB.setPwmRange(new PwmControl.PwmRange(500, 2500));


        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setTargetPosition(0);
        climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMaster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMaster.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftSlave.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climb.setPower(1);
        launchRelease.setPosition(0);
        pixelRelease.setPosition(0);
        clawFlip.setPosition(flipIn);
        intakeLink.setPosition(intakeUp);
    }
    public void driveRobotOriented(double X, double Y, double R){
        frontRight.setPower(Y-X-R);
        backRight.setPower(Y+X-R);
        frontLeft.setPower(Y+X+R);
        backLeft.setPower(Y-X+R);
    }
    public void driveSlowMo(boolean forward, boolean backward, boolean right, boolean left, double rotation){
        if(forward){
            driveRobotOriented(0, 0.25, rotation);
        }
        else if(backward){
            driveRobotOriented(0,-0.25,rotation);
        }

        if(right){
            driveRobotOriented(0.25, 0, rotation);
        }
        else if(left){
            driveRobotOriented(-0.25, 0, rotation);
        }
    }

    //intake stuff
    public void intake(double speed){
        intakeMA.setPower(speed);
        intakeSA.setPower(speed/0.8);
        intakeSB.setPower(speed/0.8);
    }
    public void intakeFlip(boolean down){
        if(down){
            intakeLink.setPosition(intakeDown);
        }
        else{
            intakeLink.setPosition(intakeUp);
        }
    }

    //lift stuff
    public void lift(int position, double timer){
            liftPosition = position;
            if(liftPosition == 0 && timer < 300){
                liftMaster.setPower(0);
                liftSlave.setPower(0);
            }
            else{
                liftPower = liftP * (position-liftMaster.getCurrentPosition());
                liftMaster.setPower(liftPower);
                liftSlave.setPower(-liftPower);
            }

    }
    public double liftPower(){return liftPower;}
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

    public void clawFlip(){
        if(liftCurrentPosition()<300 || liftPosition == home){
            clawFlip.setPosition(flipIn);
        }
        else{
            clawFlip.setPosition(flipOut);
        }
    }

    public void clawRotate(boolean up, boolean down){
        if(liftTargetPosition()==0){
            clawRotate.setPower(rotatePostion);
        }
        else{
            if(rotateTrigger && (up || down)){
                rotateTrigger = false;
                if(up){
                    rotatePostion += 1;
                }
                else{
                    rotatePostion -= 1;
                }
            }
            else{
                rotateTrigger = true;
            }
        }

        if(rotatePostion > 2){
            rotatePostion=2;
        }
        else if(rotatePostion < -2){
            rotatePostion = -2;
        }

        if(rotatePostion == 0){
            clawRotate.setPower(rotate0);
        }
        else if(rotatePostion == 1){
            clawRotate.setPower(rotate30);
        }
        else if(rotatePostion == 2){
            clawRotate.setPower(rotate90);
        }
        else if(rotatePostion == -1){
            clawRotate.setPower(-rotate30);
        }
        else if(rotatePostion == -2){
            clawRotate.setPower(-rotate90);
        }
    }

    //endgame
    public void launch(){
        launchRelease.setPosition(0.5);
    }

    public void climb(int position){
        climb.setTargetPosition(position);
        climb.setPower(1);
    }
    public int climbPosition(){
        return climb.getCurrentPosition();
    }
}