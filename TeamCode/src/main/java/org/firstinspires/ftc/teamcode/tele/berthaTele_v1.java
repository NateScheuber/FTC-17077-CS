package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.berthaHardware.home;
import static org.firstinspires.ftc.teamcode.berthaHardware.level1;
import static org.firstinspires.ftc.teamcode.berthaHardware.level2;
import static org.firstinspires.ftc.teamcode.berthaHardware.level3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.berthaHardware;

public class berthaTele_v1 extends LinearOpMode {

        public boolean claw = true;
        public boolean blocker = false;
        public boolean intakeFlip = true;
        public boolean clawClosed = false;
        public boolean clawClosedToggle = false;


    berthaHardware Bertha = new berthaHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        Bertha.init();


        waitForStart();
        while(opModeIsActive()){
            //drive
            double driveX = gamepad1.right_stick_x;
            double driveY = -gamepad1.right_stick_y;
            double driveR = gamepad1.left_stick_x + (gamepad1.right_trigger * 0.25) - (gamepad1.left_trigger * 0.25);
            Bertha.driveRobotOriented(driveX,driveY,driveR);
            Bertha.driveSlowMo(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);

            //intake
            if(gamepad1.right_bumper){
                Bertha.intake(1);
            }
            else  if(gamepad1.left_bumper){
                Bertha.intake(-1);
            }
            else{
                Bertha.intake(0);
            }

            //lift
            if(gamepad2.dpad_down){
                Bertha.lift(home);
            }
            else if(gamepad2.dpad_left){
                Bertha.lift(level1);
            }
            else if(gamepad2.dpad_right){
                Bertha.lift(level2);
            }
            else if(gamepad2.dpad_up){
                Bertha.lift(level3);
            }


            //claw open/close
            if(gamepad2.x && clawClosedToggle){
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
            else if(!gamepad2.x){
                clawClosedToggle = true;
            }

            //pixel blocker automation
            Bertha.pixelBlocker();

            //claw flip automation
            Bertha.clawFlip();

            if(gamepad1.a && gamepad2.a){
                Bertha.launch();
            }

            if(gamepad2.b){
                Bertha.climb();
            }


        }
    }
}
