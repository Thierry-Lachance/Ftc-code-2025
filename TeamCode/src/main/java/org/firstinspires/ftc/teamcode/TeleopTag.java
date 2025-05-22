package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class TeleopTag extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
    private static final boolean USE_WEBCAM = true;


    enum StateMachine {
        WAITING_FOR_TARGET,
        DRIVE_TO_TARGET_A,
        DRIVE_TO_TARGET_B,
        DRIVE_TO_TARGET_X,
        DRIVE_TO_TARGET_Y

    }

    static final Pose2D TARGET_A = new Pose2D(DistanceUnit.INCH, 40, 13, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_B = new Pose2D(DistanceUnit.INCH, 15, -15, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_X = new Pose2D(DistanceUnit.INCH, 15, 15, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_Y = new Pose2D(DistanceUnit.INCH, 15, 0, AngleUnit.DEGREES, 0);

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            4.5, 8, 7, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
   
    private static final int POSITION_1 = -75; // Preset position 1 (encoder counts)
    private static final int POSITION_2 = -3200;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");



        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(10.0, -5.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_TARGET;
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();
      //  visionPortal.stopStreaming();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();
            odo.update();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x*0.7;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (stateMachine == StateMachine.WAITING_FOR_TARGET) {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            } else {
                frontLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                frontRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                backLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                backRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        odo.setPosition(new Pose2D(DistanceUnit.INCH, detection.robotPose.getPosition().y, -detection.robotPose.getPosition().x, AngleUnit.DEGREES, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    }
                }
            } else if (gamepad1.x) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_X;
                nav.driveTo(odo.getPosition(), TARGET_X, 0.5, 0);
            } else if (gamepad1.y) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_Y;
                nav.driveTo(odo.getPosition(), TARGET_Y, 0.5, 0);
            } else if (gamepad1.a) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_A;
                nav.driveTo(odo.getPosition(), TARGET_A, 0.5, 0);
            } else if (gamepad1.b) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_B;
                nav.driveTo(odo.getPosition(), TARGET_B, 0.5, 0);
            } else {
                stateMachine = StateMachine.WAITING_FOR_TARGET;
            }

        }
        visionPortal.close();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine(String.format("x %6.1f", odo.getPosition().getX(DistanceUnit.INCH)));
        telemetry.addLine(String.format("y %6.1f", odo.getPosition().getY(DistanceUnit.INCH)));

    }   // end method telemetryAprilTag()

}   // end class
