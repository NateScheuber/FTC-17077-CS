package org.firstinspires.ftc.teamcode.auto;


import static org.firstinspires.ftc.teamcode.berthaHardware.home;
import static org.firstinspires.ftc.teamcode.berthaHardware.liftPosition;

import org.firstinspires.ftc.teamcode.berthaHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto League 2")
public class AutoParkRed extends LinearOpMode {

    berthaHardware Bertha = new berthaHardware(this);
    public ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        Bertha.init();
        autoTimer.startTime();

        waitForStart();
        autoTimer.reset();
        while(autoTimer.milliseconds()<1000){
            Bertha.driveRobotOriented(0, -0.5,0);
        }
        Bertha.driveRobotOriented(0,0,0);

    }
}
