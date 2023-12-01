package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.VIsion.Location_Pipeline;
import org.firstinspires.ftc.teamcode.berthaHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red")
public class Meet1RedAudience extends LinearOpMode {
    berthaHardware Bertha = new berthaHardware(this);

    OpenCvCamera webcam;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Location_Pipeline detector = new Location_Pipeline();
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


                Bertha.init();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startpose = new Pose2d(36, -63, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);
        telemetry.addData("Status", "Pose Set");
        telemetry.update();

        Trajectory preload = drive.trajectoryBuilder(startpose)
            .splineToConstantHeading(new Vector2d(40, 36), Math.toRadians(180))
            .build();

        Trajectory autocyclePU = drive.trajectoryBuilder(preload.end())
            .splineToConstantHeading(new Vector2d(6, -12), Math.toRadians(180))
            .splineToConstantHeading(new Vector2d(-60, -12), Math.toRadians(180))
            .build();

        Trajectory autocycleDEP = drive.trajectoryBuilder(autocyclePU.end())
            .splineToConstantHeading(new Vector2d(6, -12), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(40, -36), Math.toRadians(0))
            .build();

        Trajectory park = drive.trajectoryBuilder(autocycleDEP.end())
            .splineToConstantHeading(new Vector2d(6, -12), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
            .build();

        waitForStart();


        switch (detector.getLocation()){
            case LEFT:
                webcam.stopStreaming();

                break;
            case MIDDLE:
                webcam.stopStreaming();

                break;
            case RIGHT:
                webcam.stopStreaming();

                break;
        }




    }
}
