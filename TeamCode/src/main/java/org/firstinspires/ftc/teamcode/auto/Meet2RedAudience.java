package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.VIsion.Location_Pipeline_Red;
import org.firstinspires.ftc.teamcode.berthaHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Red w/vision")
public class Meet2RedAudience extends LinearOpMode {
    berthaHardware Bertha = new berthaHardware(this);

    OpenCvCamera webcam;
    public int position = 1;


    @Override
    public void runOpMode() throws InterruptedException {
                Bertha.init();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startpose = new Pose2d(12, -63, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);
        telemetry.addData("Status", "Pose Set");
        telemetry.update();

        //Position 1
        Trajectory preload1 = drive.trajectoryBuilder(startpose)
                .splineToSplineHeading(new Pose2d(52, -30, Math.toRadians(180)), Math.toRadians(10))
                .build();

        Trajectory placeYellow1 = drive.trajectoryBuilder(preload1.end())
                .splineToSplineHeading(new Pose2d(12, -28, Math.toRadians(0)),Math.toRadians(180))
                .build();

        Trajectory park1 = drive.trajectoryBuilder(placeYellow1.end())
                .splineToSplineHeading(new Pose2d(52, -62,Math.toRadians(0)), Math.toRadians(0))
                .build();


        //Position 2
        Trajectory preload2 = drive.trajectoryBuilder(startpose)
                .splineToSplineHeading(new Pose2d(52, -36, Math.toRadians(180)), Math.toRadians(10))
                .build();

        Trajectory placeYellow2 = drive.trajectoryBuilder(preload2.end())
                .splineToSplineHeading(new Pose2d(12, -34, Math.toRadians(-90)),Math.toRadians(200))
                .build();

        Trajectory park2 = drive.trajectoryBuilder(placeYellow2.end())
                .splineToSplineHeading(new Pose2d(52, -62, Math.toRadians(0)), Math.toRadians(0))
                .build();


        //Position 3
        Trajectory preload3 = drive.trajectoryBuilder(startpose)
                .splineToSplineHeading(new Pose2d(52, -42, Math.toRadians(180)), Math.toRadians(10))
                .build();

        Trajectory placeYellow3 = drive.trajectoryBuilder(preload3.end())
                .splineToSplineHeading(new Pose2d(28, -36, Math.toRadians(-90)),Math.toRadians(-150))
                .build();

        Trajectory park3 = drive.trajectoryBuilder(placeYellow3.end())
                .splineToSplineHeading(new Pose2d(52, -62,Math.toRadians(0)), Math.toRadians(0))
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Location_Pipeline_Red detector = new Location_Pipeline_Red(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        while(!isStarted()){
            position = Location_Pipeline_Red.position();
        }

        waitForStart();
        if(position == 1){
            telemetry.addData("Trajectory", "1");
            telemetry.update();
        }
        else if(position == 2){
            telemetry.addData("Trajectory", "2");
            telemetry.update();
        }
        else{
            telemetry.addData("Trajectory", "3");
            telemetry.update();
        }





    }
}
