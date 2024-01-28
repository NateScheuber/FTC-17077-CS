package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.berthaHardware.flipIn;
import static org.firstinspires.ftc.teamcode.berthaHardware.flipOut;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotate0;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotateCCW90;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.VIsion.Location_Pipeline_Blue;
import org.firstinspires.ftc.teamcode.berthaHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Backboard")
public class BlueBackboardNoAutocycle extends LinearOpMode {

    berthaHardware Bertha = new berthaHardware(this);

    OpenCvCamera webcam;
    public int position = 1;
    public double distanceX = 0;


    @Override
    public void runOpMode() throws InterruptedException {


        Bertha.init();

        Rev2mDistanceSensor distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        DcMotorEx liftMaster = hardwareMap.get(DcMotorEx.class, "liftMaster");
        DcMotorEx liftSlave = hardwareMap.get(DcMotorEx.class, "liftSlave");
        liftMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftSlave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx intakeMA = hardwareMap.get(DcMotorEx.class, "intakeMA");
        ServoImplEx intakeLinkA = hardwareMap.get(ServoImplEx.class, "intakeLinkA");
        ServoImplEx intakeLinkB = hardwareMap.get(ServoImplEx.class, "intakeLinkB");
        CRServoImplEx intakeSA = hardwareMap.get(CRServoImplEx.class, "intakeSA");
        CRServoImplEx intakeSB = hardwareMap.get(CRServoImplEx.class, "intakeSB");
        intakeSA.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLinkB.setDirection(Servo.Direction.REVERSE);

        CRServoImplEx clawRotate = hardwareMap.get(CRServoImplEx.class, "clawRotate");
        ServoImplEx clawFlip = hardwareMap.get(ServoImplEx.class, "clawFlip");
        intakeMA.setDirection(DcMotorSimple.Direction.REVERSE);
        clawFlip.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawRotate.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakeLinkA.setPosition(0.65);
        intakeLinkB.setPosition(0.65);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(8.3, 63, Math.toRadians(90));

        Trajectory placePurple1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12,36, Math.toRadians(0)), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    intakeSA.setPower(-1);
                    intakeSB.setPower(-1);
                })
                .build();

        Trajectory placePurple2 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(24,24,Math.toRadians(180)), Math.toRadians(-90))
                .addDisplacementMarker(()->{
                    intakeSA.setPower(-1);
                    intakeSB.setPower(-1);
                })
                .build();

        Trajectory placePurple3 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(36,42,Math.toRadians(180)), Math.toRadians(-90))
                .addDisplacementMarker(()->{
                    intakeSA.setPower(-1);
                    intakeSB.setPower(-1);
                })
                .build();


        Trajectory placeYellow1 = drive.trajectoryBuilder(placePurple1.end())
                .addTemporalMarker(0.5,()->{
                    intakeSA.setPower(-1);
                    intakeSB.setPower(-1);
                })
                .addTemporalMarker(0.65, () -> {
                    liftMaster.setPower(1);
                    liftSlave.setPower(1);
                })
                .addTemporalMarker(0.9, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                    intakeMA.setPower(0);
                    clawFlip.setPosition(flipOut);
                    clawRotate.setPower(rotateCCW90);
                })
                .lineToLinearHeading(new Pose2d(50,30,Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    Bertha.closeClaw(false);
                })
                .build();

        Trajectory placeYellow2 = drive.trajectoryBuilder(placePurple2.end())
                .addTemporalMarker(0.5,()->{
                    intakeSA.setPower(-1);
                    intakeSB.setPower(-1);
                })
                .addTemporalMarker(0.65, () -> {
                    liftMaster.setPower(1);
                    liftSlave.setPower(1);
                })
                .addTemporalMarker(0.9, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                    intakeMA.setPower(0);
                    clawFlip.setPosition(flipOut);
                    clawRotate.setPower(rotateCCW90);
                })
                .lineToLinearHeading(new Pose2d(50,30,Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    Bertha.closeClaw(false);
                })
                .build();

        Trajectory placeYellow3 = drive.trajectoryBuilder(placePurple3.end())
                .addTemporalMarker(0.5,()->{
                    intakeSA.setPower(-1);
                    intakeSB.setPower(-1);
                })
                .addTemporalMarker(0.65, () -> {
                    liftMaster.setPower(1);
                    liftSlave.setPower(1);
                })
                .addTemporalMarker(0.9, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                    intakeMA.setPower(0);
                    clawFlip.setPosition(flipOut);
                    clawRotate.setPower(rotateCCW90);
                })
                .lineToLinearHeading(new Pose2d(50,30,Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    Bertha.closeClaw(false);
                })
                .build();

        Trajectory park1 = drive.trajectoryBuilder(placeYellow1.end())
                .addTemporalMarker(0.3,()->{
                    clawFlip.setPosition(flipIn);
                    clawRotate.setPower(rotate0);
                })
                .addTemporalMarker(0.5, () -> {
                    liftMaster.setPower(-0.5);
                    liftSlave.setPower(-0.5);
                })
                .addTemporalMarker(0.9, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50,62), Math.toRadians(180))
                .build();

        Trajectory park2 = drive.trajectoryBuilder(placeYellow2.end())
                .addTemporalMarker(0.3,()->{
                    clawFlip.setPosition(flipIn);
                    clawRotate.setPower(rotate0);
                })
                .addTemporalMarker(0.5, () -> {
                    liftMaster.setPower(-0.5);
                    liftSlave.setPower(-0.5);
                })
                .addTemporalMarker(0.9, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50,62), Math.toRadians(180))
                .build();

        Trajectory park3 = drive.trajectoryBuilder(placeYellow3.end())
                .addTemporalMarker(0.3,()->{
                    clawFlip.setPosition(flipIn);
                    clawRotate.setPower(rotate0);
                })
                .addTemporalMarker(0.5, () -> {
                    liftMaster.setPower(-0.5);
                    liftSlave.setPower(-0.5);
                })
                .addTemporalMarker(0.9, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50,62), Math.toRadians(180))
                .build();

        telemetry.addData("Path", "Built");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Location_Pipeline_Blue detector = new Location_Pipeline_Blue(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        while(!isStarted()){
            position = Location_Pipeline_Blue.position();
        }

        if(position == 1){
            drive.followTrajectory(placePurple1);
            drive.followTrajectory(placeYellow1);
            drive.followTrajectory(park1);
        }
        else if(position == 2){
            drive.followTrajectory(placePurple2);
            drive.followTrajectory(placeYellow2);
            drive.followTrajectory(park2);
        }
        else{
            drive.followTrajectory(placePurple3);
            drive.followTrajectory(placeYellow3);
            drive.followTrajectory(park3);
        }
    }
}
