package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.berthaHardware.home;
import static org.firstinspires.ftc.teamcode.berthaHardware.level1;
import static org.firstinspires.ftc.teamcode.berthaHardware.level2;
import static org.firstinspires.ftc.teamcode.berthaHardware.level3;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.berthaHardware;
@Config
@TeleOp(name = "berthaTele")
public class berthaTele_v1 extends LinearOpMode {

        public boolean claw = true;
        public boolean blocker = false;
        public boolean intakeFlip = true;
        public boolean clawClosed = false;
        public boolean clawClosedToggle = false;
        public boolean rotateTrigger = false;



        public static int liftPosition = 0;
        public ElapsedTime liftDelay = new ElapsedTime();


    berthaHardware Bertha = new berthaHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {

        Rev2mDistanceSensor distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        Bertha.init();

        waitForStart();
        while(opModeIsActive()){
            //drive
            double driveX = gamepad1.right_stick_x;
            double driveY = -gamepad1.right_stick_y;
            double driveR = gamepad1.left_stick_x + (gamepad1.right_trigger * 0.25) - (gamepad1.left_trigger * 0.25);
            Bertha.driveRobotOriented(
                    driveX,
                    driveY,
                    driveR);
            Bertha.driveSlowMo(
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    gamepad1.dpad_right,
                    gamepad1.dpad_left,
                    (gamepad1.right_trigger * 0.25) - (gamepad1.left_trigger * 0.25));

            //intake
            if(gamepad1.right_bumper){
                Bertha.intake(0.8);
                Bertha.intakeFlip(true);
            }
            else  if(gamepad1.left_bumper){
                Bertha.intake(-0.3);
                Bertha.intakeFlip(false);
            }
            else{
                Bertha.intake(0);
            }

            //lift
            if(gamepad2.dpad_down){
                liftDelay.reset();
                liftPosition = home;
            }
            else if(gamepad2.dpad_left){
                liftPosition = level1;
            }
            else if(gamepad2.dpad_right){
                liftPosition = level2;
            }
            else if(gamepad2.dpad_up){
                liftPosition = level3;
            }

            Bertha.lift((int) (liftPosition+(150*gamepad2.right_trigger)-(150*gamepad2.left_trigger)), liftDelay.milliseconds());

            //claw open/close
            if(gamepad2.cross && clawClosedToggle){
                clawClosedToggle = false;
                if(clawClosed){
                    Bertha.closeClaw(false);
                    clawClosed = false;
                }
                else{
                    Bertha.closeClaw(true);
                    clawClosed = true;
                }
            }
            else if(!gamepad2.cross){
                clawClosedToggle = true;
            }

            Bertha.clawRotate(gamepad2.right_bumper, gamepad2.left_bumper);


            //claw flip automation
            Bertha.clawFlip();

            if(gamepad1.a && gamepad2.a){
                Bertha.launch();
            }

            if(gamepad1.y){
                Bertha.climb(-8000);
            }
            else if(gamepad1.x){
                Bertha.climb(-3500);
            }

            
            telemetry.addData("Lift Power", Bertha.liftPower());
            telemetry.addData("Lift Current Position", Bertha.liftCurrentPosition());
            telemetry.addData("Climb Current Position", Bertha.climbPosition());
            telemetry.update();
        }
    }
}