package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.berthaHardware;

public class berthaTele_v1 extends LinearOpMode {

        public boolean claw = true;
        public boolean blocker = false;
        public boolean intakeFlip = true;

    berthaHardware Bertha = new berthaHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        Bertha.init();


        waitForStart();
        while(opModeIsActive()){
            double driveX = gamepad1.right_stick_x;
            double driveY = -gamepad1.right_stick_y;
            double driveR = gamepad1.left_stick_x + (gamepad1.right_trigger * 0.25) - (gamepad1.left_trigger * 0.25);
            Bertha.driveRobotOriented(driveX,driveY,driveR);
            Bertha.driveSlowMo(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);


            if(gamepad1.right_bumper){
                Bertha.intake(1);
            }
            else  if(gamepad1.left_bumper){
                Bertha.intake(-1);
            }
            else{
                Bertha.intake(0);
            }




        }
    }
}
