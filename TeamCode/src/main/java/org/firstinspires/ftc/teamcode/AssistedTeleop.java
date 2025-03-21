package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class AssistedTeleop extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_TARGET,
        DRIVE_TO_TARGET_A,
        DRIVE_TO_TARGET_B,
        DRIVE_TO_TARGET_X,
        DRIVE_TO_TARGET_Y

    }
    static final Pose2D TARGET_A = new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_B = new Pose2D(DistanceUnit.CM, 60, -60, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_X = new Pose2D(DistanceUnit.CM,60,60, AngleUnit.DEGREES,0);
    static final Pose2D TARGET_Y = new Pose2D(DistanceUnit.CM, 60, 0, AngleUnit.DEGREES, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("flMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("blMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("brMotor");


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
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(10.0, -5.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

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

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            odo.update();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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

            if(stateMachine == StateMachine.WAITING_FOR_TARGET){
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }
            else {
                frontLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                frontRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                backLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                backRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }
            if(gamepad1.right_bumper || gamepad1.left_bumper){
                stateMachine = StateMachine.WAITING_FOR_TARGET;
            }
            else if(gamepad1.x){
                stateMachine = StateMachine.DRIVE_TO_TARGET_X;
                nav.driveTo(odo.getPosition(), TARGET_X, 0.5, 0);
            }
            else if(gamepad1.y){
                stateMachine = StateMachine.DRIVE_TO_TARGET_Y;
                nav.driveTo(odo.getPosition(), TARGET_Y, 0.5, 0);
            }
            else if(gamepad1.a){
                stateMachine = StateMachine.DRIVE_TO_TARGET_A;
                nav.driveTo(odo.getPosition(), TARGET_A, 0.5, 0);
            }
            else if(gamepad1.b){
                stateMachine = StateMachine.DRIVE_TO_TARGET_B;
                nav.driveTo(odo.getPosition(), TARGET_B, 0.5, 0);
            }
            else{
                stateMachine = StateMachine.WAITING_FOR_TARGET;
            }
        }
    }
}
