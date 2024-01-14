package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import static org.firstinspires.ftc.teamcode.berthaHardware.flipIn;
import static org.firstinspires.ftc.teamcode.berthaHardware.flipOut;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotate0;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotateCCW90;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotateCW90;

@Config
@Autonomous(name = "Blue w/vision")
public class InterleaguesBlueAudience extends LinearOpMode {
    berthaHardware Bertha = new berthaHardware(this);

    OpenCvCamera webcam;
    public int position = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Bertha.init();
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

        Pose2d startpose = new Pose2d(-39.7, 63, Math.toRadians(90));
        drive.setPoseEstimate(startpose);
        telemetry.addData("Status", "Pose Set");
        telemetry.update();

        //Position 1
        Trajectory placePurple1 = drive.trajectoryBuilder(startpose, true)
                .splineToConstantHeading(new Vector2d(-39.7, 50), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-32, 32, Math.toRadians(135)), Math.toRadians(-45))
                .build();

        Trajectory intialIntake1 = drive.trajectoryBuilder(placePurple1.end())
                .addDisplacementMarker(()->{
                    intakeLinkA.setPosition(0.47);
                    intakeLinkB.setPosition(0.47);
                    intakeMA.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-47, 37, Math.toRadians(-180)),Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-63, 38), Math.toRadians(-180))
                .addDisplacementMarker(()->{
                    intakeLinkA.setPosition(0.65);
                    intakeLinkB.setPosition(0.65);
                })
                .build();

        Trajectory initialScore1 = drive.trajectoryBuilder(intialIntake1.end(), true)
                .addTemporalMarker(1.25,()->{
                    intakeMA.setPower(0);
                    Bertha.closeClaw(true);
                })
                .addTemporalMarker(1.75, () -> {
                    liftMaster.setPower(1);
                    liftSlave.setPower(1);
                })
                .addTemporalMarker(2.0, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                    intakeMA.setPower(0);
                    clawFlip.setPosition(flipOut);
                    clawRotate.setPower(rotateCCW90);
                })
                .splineToConstantHeading(new Vector2d(-24, 38), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, 38), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(34, 44), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Bertha.closeClaw(false);
                })
                .build();

        Trajectory park1 = drive.trajectoryBuilder(initialScore1.end())
                .addTemporalMarker(0.3,()->{
                    clawFlip.setPosition(flipIn);
                    clawRotate.setPower(rotate0);
                })
                .addTemporalMarker(0.5, () -> {
                    liftMaster.setPower(-0.5);
                    liftSlave.setPower(-0.5);
                })
                .addTemporalMarker(0.75, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(25, 33), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(38, 16), Math.toRadians(0))
                .build();
        telemetry.addData("PATH 1", "Complete");
        telemetry.update();


        //Position 2
        Trajectory placePurple2 = drive.trajectoryBuilder(startpose, true)
                .splineToConstantHeading(new Vector2d(-38, 31), Math.toRadians(-90))
                .build();

        Trajectory initialIntake2 = drive.trajectoryBuilder(placePurple2.end())
                .addDisplacementMarker(()->{
                    intakeLinkA.setPosition(0.47);
                    intakeLinkB.setPosition(0.47);
                    intakeMA.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-47, 44, Math.toRadians(-180)),Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-61, 38), Math.toRadians(-180))
                .addDisplacementMarker(()->{
                    intakeLinkA.setPosition(0.65);
                    intakeLinkB.setPosition(0.65);
                })
                .build();

        Trajectory initialScore2 = drive.trajectoryBuilder(initialIntake2.end(), true)
                .addTemporalMarker(1.25,()->{
                    intakeMA.setPower(0);
                    Bertha.closeClaw(true);
                })
                .addTemporalMarker(1.75, () -> {
                    liftMaster.setPower(1);
                    liftSlave.setPower(1);
                })
                .addTemporalMarker(2.0, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                    intakeMA.setPower(0);
                    clawFlip.setPosition(flipOut);
                    clawRotate.setPower(rotateCCW90);
                })
                .splineToConstantHeading(new Vector2d(-24, 37), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, 37), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, 38), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Bertha.closeClaw(false);
                })
                .build();

        Trajectory park2 = drive.trajectoryBuilder(initialScore2.end())
                .addTemporalMarker(0.3,()->{
                    clawFlip.setPosition(flipIn);
                    clawRotate.setPower(rotate0);
                })
                .addTemporalMarker(0.5, () -> {
                    liftMaster.setPower(-0.5);
                    liftSlave.setPower(-0.5);
                })
                .addTemporalMarker(0.75, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(25, 33), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(35, 16), Math.toRadians(0))
                .build();

        telemetry.addData("PATH 2", "Complete");
        telemetry.update();



        //Position 3
        Trajectory placePurple3 = drive.trajectoryBuilder(startpose, true)
                .splineToConstantHeading(new Vector2d(-47, 32), Math.toRadians(-90))
                .build();

        Trajectory initialIntake3 = drive.trajectoryBuilder(placePurple3.end())
                .addDisplacementMarker(()->{
                    intakeLinkA.setPosition(0.47);
                    intakeLinkB.setPosition(0.47);
                    intakeMA.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-52, 44, Math.toRadians(-180)),Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-61, 38), Math.toRadians(-180))
                .addDisplacementMarker(()->{
                    intakeLinkA.setPosition(0.65);
                    intakeLinkB.setPosition(0.65);
                })
                .build();

        Trajectory initialScore3 = drive.trajectoryBuilder(initialIntake3.end(), true)
                .addTemporalMarker(1.25,()->{
                    intakeMA.setPower(0);
                    Bertha.closeClaw(true);
                })
                .addTemporalMarker(1.75, () -> {
                    liftMaster.setPower(1);
                    liftSlave.setPower(1);
                })
                .addTemporalMarker(2.0, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                    intakeMA.setPower(0);
                    clawFlip.setPosition(flipOut);
                    clawRotate.setPower(rotateCCW90);
                })
                .splineToConstantHeading(new Vector2d(-24, 37), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-5, 37), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(34, 33.5), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Bertha.closeClaw(false);
                })
                .build();

        Trajectory park3 = drive.trajectoryBuilder(initialScore3.end())
                .addTemporalMarker(0.3,()->{
                    clawFlip.setPosition(flipIn);
                    clawRotate.setPower(rotate0);
                })
                .addTemporalMarker(0.5, () -> {
                    liftMaster.setPower(-0.5);
                    liftSlave.setPower(-0.5);
                })
                .addTemporalMarker(0.75, () -> {
                    liftMaster.setPower(0);
                    liftSlave.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(25, 33), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(38, 16), Math.toRadians(0))
                .build();
        telemetry.addData("PATH 3", "Complete");
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Location_Pipeline_Blue detector = new Location_Pipeline_Blue(telemetry);
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
            position = Location_Pipeline_Blue.position();
        }
        webcam.stopStreaming();
        position = Location_Pipeline_Blue.position();

        if(position == 1){
            telemetry.addData("Running", "1");
            telemetry.update();
            drive.followTrajectory(placePurple1);
            drive.followTrajectory(intialIntake1);
            //drive.followTrajectory(initialScore1);
            //drive.followTrajectory(park1);
        }
        else if(position == 2){
            telemetry.addData("Running", "2");
            telemetry.update();
            drive.followTrajectory(placePurple2);
            drive.followTrajectory(initialIntake2);
            //drive.followTrajectory(initialScore2);
            //drive.followTrajectory(park2);
        }
        else{
            telemetry.addData("Running", "3");
            telemetry.update();
            drive.followTrajectory(placePurple3);
            drive.followTrajectory(initialIntake3);
            //drive.followTrajectory(initialScore3);
            //drive.followTrajectory(park3);
        }
    }
}
