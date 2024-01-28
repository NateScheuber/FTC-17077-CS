package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.berthaHardware.flipIn;
import static org.firstinspires.ftc.teamcode.berthaHardware.flipOut;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotate0;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotateCCW90;
import static org.firstinspires.ftc.teamcode.berthaHardware.rotateCW90;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.VIsion.Location_Pipeline_Blue;
import org.firstinspires.ftc.teamcode.berthaHardware;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "Blue Audience AutoCycle", group = "AutoCycle")
public class v2BlueAudienceAutocycle extends LinearOpMode {

    berthaHardware Bertha = new berthaHardware(this);

    OpenCvCamera webcam;
    public int position = 1;
    public static double scoreX = 55;
    public static double scoreY = 34;

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

        Pose2d startPose = new Pose2d(-39.7, 63, Math.toRadians(90));
        Pose2d scorePose = new Pose2d(scoreX, scoreY, Math.toRadians(180));

        drive.setPoseEstimate(startPose);
        telemetry.addData("Status", "Pose Set");
        telemetry.update();

        //Position 3
        Trajectory placePurple3 = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(-54, 32, Math.toRadians(135)))
                .build();

        Trajectory initialIntake3 = drive.trajectoryBuilder(placePurple3.end())
                .addDisplacementMarker(() -> {
                    intakeSA.setPower(0);
                    intakeSB.setPower(0);
                    intakeLinkA.setPosition(0.48);
                    intakeLinkB.setPosition(0.48);
                    intakeMA.setPower(1);
                })
                .splineTo(new Vector2d(-62, 36), Math.toRadians(181))
                .build();

        //Position 2
        Trajectory placePurple2 = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(-45, 31, Math.toRadians(135)))
                .build();

        Trajectory initialIntake2 = drive.trajectoryBuilder(placePurple2.end())
                .addDisplacementMarker(() -> {
                    intakeSA.setPower(0);
                    intakeSB.setPower(0);
                    intakeLinkA.setPosition(0.48);
                    intakeLinkB.setPosition(0.48);
                    intakeMA.setPower(1);
                })
                .splineTo(new Vector2d(-63, 36), Math.toRadians(181), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //Position 1
        Trajectory placePurple1 = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-33, 34), Math.toRadians(-45))
                .build();

        Trajectory initialIntake1 = drive.trajectoryBuilder(placePurple1.end())
                .addDisplacementMarker(() -> {
                    intakeSA.setPower(0);
                    intakeSB.setPower(0);
                    intakeLinkA.setPosition(0.48);
                    intakeLinkB.setPosition(0.48);
                    intakeMA.setPower(1);
                })
                .splineTo(new Vector2d(-64, 36), Math.toRadians(180.5), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory initialSetup1 = drive.trajectoryBuilder(initialIntake1.end(), -90)
                .addTemporalMarker(0.3,() -> {
                    intakeSA.setPower(-0.5);
                    intakeSB.setPower(-0.5);
                    intakeLinkA.setPosition(0.65);
                    intakeLinkB.setPosition(0.65);
                    intakeMA.setPower(-0.5);
                    Bertha.closeClaw(true);
                })
                .addTemporalMarker(1,() -> {
                    intakeSA.setPower(0);
                    intakeSB.setPower(0);
                    intakeMA.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-60, 36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-40,18), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(10,18), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24,18), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(26, 42), Math.toRadians(-270),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory autoIntake1 = drive.trajectoryBuilder(new Pose2d(scorePose.getX(), scorePose.getY()+8, scorePose.getHeading()))
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
                .splineToConstantHeading(new Vector2d(53, 32), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(24, 14), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-47, 14), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    intakeSA.setPower(1);
                    intakeSB.setPower(1);
                    intakeLinkA.setPosition(0.43);
                    intakeLinkB.setPosition(0.43);
                    intakeMA.setPower(1);
                })
                .splineToConstantHeading(new Vector2d(-54,14), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //same same
        Trajectory initialSetup = drive.trajectoryBuilder(initialIntake3.end(), -90)
                .addTemporalMarker(0.3,() -> {
                    intakeSA.setPower(-0.5);
                    intakeSB.setPower(-0.5);
                    intakeLinkA.setPosition(0.65);
                    intakeLinkB.setPosition(0.65);
                    intakeMA.setPower(-0.5);
                    Bertha.closeClaw(true);
                })
                .addTemporalMarker(1.5,() -> {
                    intakeSA.setPower(0);
                    intakeSB.setPower(0);
                    intakeMA.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-61, 36), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-61, 24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(0,16), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24,16), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(26, 34), Math.toRadians(-270),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory autoIntake = drive.trajectoryBuilder(scorePose)
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
                .splineToConstantHeading(new Vector2d(53, 32), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(24, 13), Math.toRadians(180))
                .splineTo(new Vector2d(-47, 13), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    intakeSA.setPower(1);
                    intakeSB.setPower(1);
                    intakeLinkA.setPosition(0.43);
                    intakeLinkB.setPosition(0.43);
                    intakeMA.setPower(1);
                })
                .splineToConstantHeading(new Vector2d(-54,13), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory autoScore = drive.trajectoryBuilder(autoIntake.end(), true)
                .addTemporalMarker(0.5,() -> {
                    intakeSA.setPower(-0.5);
                    intakeSB.setPower(-0.5);
                    intakeLinkA.setPosition(0.65);
                    intakeLinkB.setPosition(0.65);
                    intakeMA.setPower(-0.5);
                    Bertha.closeClaw(true);
                })
                .addTemporalMarker(1,() -> {
                    intakeSA.setPower(0);
                    intakeSB.setPower(0);
                    intakeMA.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(15,14), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36,14), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(40, 36), Math.toRadians(-270),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory park = drive.trajectoryBuilder(scorePose)
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
                .forward(9, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
        webcam.stopStreaming();

        waitForStart();

        if(position == 3) {
            sleep(3000);
            drive.followTrajectory(placePurple3);
            drive.followTrajectory(initialIntake3);
            drive.followTrajectory(initialSetup);
            distanceX = distance.getDistance(DistanceUnit.INCH);
            Trajectory initialScore1 = drive.trajectoryBuilder(initialSetup.end())
                    .addTemporalMarker(0.0, () -> {
                        liftMaster.setPower(1);
                        liftSlave.setPower(1);
                    })
                    .addTemporalMarker(0.25, () -> {
                        liftMaster.setPower(0);
                        liftSlave.setPower(0);
                        intakeMA.setPower(0);
                        clawFlip.setPosition(flipOut);
                        clawRotate.setPower(rotateCW90);
                    })
                    .back(distanceX+2.5, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(()->{
                        Bertha.closeClaw(false);
                    })
                    .build();
            drive.followTrajectory(initialScore1);
            drive.setPoseEstimate(new Pose2d(scoreX, scoreY, Math.toRadians(180)));
            drive.followTrajectory(autoIntake);
            drive.followTrajectory(autoScore);
            distanceX = distance.getDistance(DistanceUnit.INCH);
            Trajectory initialScore2 = drive.trajectoryBuilder(autoScore.end())
                    .addTemporalMarker(0.0, () -> {
                        liftMaster.setPower(1);
                        liftSlave.setPower(1);
                    })
                    .addTemporalMarker(0.3, () -> {
                        liftMaster.setPower(0);
                        liftSlave.setPower(0);
                        intakeMA.setPower(0);
                        clawFlip.setPosition(flipOut);
                        clawRotate.setPower(rotateCW90);
                    })
                    .back(distanceX+2.2, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(()->{
                        Bertha.closeClaw(false);
                    })
                    .build();
            drive.followTrajectory(initialScore2);
            drive.setPoseEstimate(new Pose2d(scoreX, scoreY, Math.toRadians(180)));
            drive.followTrajectory(park);
        }
        else if(position == 2) {
            sleep(3000);
            drive.followTrajectory(placePurple2);
            drive.followTrajectory(initialIntake2);
            drive.setPoseEstimate(initialIntake3.end());
            drive.followTrajectory(initialSetup);
            distanceX = distance.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance", distanceX);
            telemetry.update();
            Trajectory initialScore1 = drive.trajectoryBuilder(initialSetup.end())
                    .addTemporalMarker(0.0, () -> {
                        liftMaster.setPower(1);
                        liftSlave.setPower(1);
                    })
                    .addTemporalMarker(0.25, () -> {
                        liftMaster.setPower(0);
                        liftSlave.setPower(0);
                        intakeMA.setPower(0);
                        clawFlip.setPosition(flipOut);
                        clawRotate.setPower(rotateCCW90);
                    })
                    .back(distanceX+3.1, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(()->{
                        Bertha.closeClaw(false);
                    })
                    .build();
            drive.followTrajectory(initialScore1);
            drive.setPoseEstimate(new Pose2d(scoreX, scoreY, Math.toRadians(180)));
            drive.followTrajectory(autoIntake);
            drive.followTrajectory(autoScore);
            distanceX = distance.getDistance(DistanceUnit.INCH);
            Trajectory initialScore2 = drive.trajectoryBuilder(autoScore.end())
                    .addTemporalMarker(0.0, () -> {
                        liftMaster.setPower(1);
                        liftSlave.setPower(1);
                    })
                    .addTemporalMarker(0.3, () -> {
                        liftMaster.setPower(0);
                        liftSlave.setPower(0);
                        intakeMA.setPower(0);
                        clawFlip.setPosition(flipOut);
                        clawRotate.setPower(rotateCW90);
                    })
                    .back(distanceX+2.7, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(()->{
                        Bertha.closeClaw(false);
                    })
                    .build();
            drive.followTrajectory(initialScore2);
            drive.setPoseEstimate(new Pose2d(scoreX, scoreY, Math.toRadians(180)));
            drive.followTrajectory(park);
        }
        else{
            sleep(3000);
            drive.followTrajectory(placePurple1);
            drive.followTrajectory(initialIntake1);
            drive.followTrajectory(initialSetup1);
            distanceX = distance.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance", distanceX);
            telemetry.update();
            Trajectory initialScore1 = drive.trajectoryBuilder(initialSetup1.end())
                    .addTemporalMarker(0.0, () -> {
                        liftMaster.setPower(1);
                        liftSlave.setPower(1);
                    })
                    .addTemporalMarker(0.25, () -> {
                        liftMaster.setPower(0);
                        liftSlave.setPower(0);
                        intakeMA.setPower(0);
                        clawFlip.setPosition(flipOut);
                        clawRotate.setPower(rotateCCW90);
                    })
                    .back(distanceX+3.1, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(()->{
                        Bertha.closeClaw(false);
                    })
                    .build();
            drive.followTrajectory(initialScore1);
            drive.setPoseEstimate(new Pose2d(scoreX, scoreY+9, Math.toRadians(180)));
            drive.followTrajectory(autoIntake1);
            drive.followTrajectory(autoScore);
            distanceX = distance.getDistance(DistanceUnit.INCH);
            Trajectory initialScore2 = drive.trajectoryBuilder(autoScore.end())
                    .addTemporalMarker(0.0, () -> {
                        liftMaster.setPower(1);
                        liftSlave.setPower(1);
                    })
                    .addTemporalMarker(0.3, () -> {
                        liftMaster.setPower(0);
                        liftSlave.setPower(0);
                        intakeMA.setPower(0);
                        clawFlip.setPosition(flipOut);
                        clawRotate.setPower(rotateCW90);
                    })
                    .back(distanceX+2.4, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(()->{
                        Bertha.closeClaw(false);
                    })
                    .build();
            drive.followTrajectory(initialScore2);
            drive.setPoseEstimate(new Pose2d(scoreX, scoreY, Math.toRadians(180)));
            drive.followTrajectory(park);
        }
    }
}
