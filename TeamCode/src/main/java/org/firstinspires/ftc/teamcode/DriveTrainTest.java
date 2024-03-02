package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.Camera;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;

@TeleOp(name="DriveTrainTest", group="Linear OpMode")
@Config
public class DriveTrainTest extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    public static double distance;
    public static double setpoint;
    public static double angle;
    public static double kp;
    public static double ki;
    public static double kd;
    private final double[] PidConstantsAngle = new double[]{1, 200, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.01, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Camera camera = new Camera();


    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        camera.init(hardwareMap);
        camera.setPipeline("red");
        FtcDashboard.getInstance().startCameraStream(camera.openStream(), 0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        boolean toggle = false;
        boolean red = true;
        waitForStart();
        while (opModeIsActive()) {
            if (toggle) {
                dt.FieldCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            } else {
                dt.robotCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            if (gamepad1.start) {
                toggle = !toggle;
            }
            if (gamepad1.a) {
                dt.getToAngle(PidConstantsAngle, angle);
            }
            if (gamepad1.b) {
                dt.driveToLocation(PidConstantsDistance,PidConstantsDistance,angle,distance);

            }
            if (gamepad1.back && gamepad1.dpad_up) {
                dt.reInitFieldCentric();
            }
            if (gamepad2.left_stick_y != 0) {
                lin.moveLift(gamepad2.left_stick_y);
            } else {
                lin.setPower(0);
            }
            if (gamepad2.a) {
                lin.gotoPosition(setpoint);
            }
            if(red){
                camera.setPipeline("red");
            }else{
                camera.setPipeline("blue");
            }
            if(gamepad2.b){
                red = !red;
            }
            packet.put("Angle", dt.getImu().getAngularOrientation().firstAngle);
            packet.put("X", dt.getxOdom().getCurrentPosition());
            packet.put("Y", dt.getyOdom().getCurrentPosition());
            packet.put("Lin", lin.getPos());
            packet.put("Red", red);
            dashboard.sendTelemetryPacket(packet);
            updateTelemetry(telemetry);
        }
    }
}

