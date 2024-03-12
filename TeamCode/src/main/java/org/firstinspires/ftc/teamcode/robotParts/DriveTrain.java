package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;


public class DriveTrain {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private BNO055IMU imu;
    private DcMotor xOdom;
    private DcMotor yOdom;
    HardwareMap hwMap;
    private double[] xPID;
    private double[] yPID;
    private double[] anglePID;

    public void init(HardwareMap hwMap, double[] xPID, double[] yPID, double[] anglePID){
        this.hwMap = hwMap;

        this.fl = hwMap.get(DcMotor.class, "fl");
        this.fr = hwMap.get(DcMotor.class, "fr");
        this.bl = hwMap.get(DcMotor.class, "bl");
        this.br = hwMap.get(DcMotor.class, "br");
        this.imu = hwMap.get(BNO055IMU.class, "imu");
        this.xOdom = hwMap.get(DcMotor.class, "odo1");
        this.yOdom = hwMap.get(DcMotor.class, "odo2");

        this.xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.xOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bl.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);

        this.xPID = xPID;
        this.yPID = yPID;
        this.anglePID = anglePID;
    }
    public void reInitFieldCentric(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);
    }
    public void robotCentricDrive(double leftStickY, double leftStickX, double rightStickX){
        double theta = Math.atan2(-1 * leftStickY, leftStickX);
        double power = Math.hypot(leftStickX, -1 * leftStickY);
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        this.fl.setPower(power * cos/max + rightStickX);
        this.fr.setPower(power * sin/max - rightStickX);
        this.bl.setPower(power * sin/max + rightStickX);
        this.br.setPower(power * cos/max - rightStickX);
    }
    public void FieldCentricDrive(double leftStickY, double leftStickX, double rightStickX, double slow){
        double y = -1 * leftStickY; // Remember, this is reversed!
        double x = leftStickX * 1.1; // Counteract imperfect strafing


        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rightStickX), 1);
        double frontLeftPower = (rotY + rotX + rightStickX) / (denominator * slow);
        double backLeftPower = (rotY - rotX + rightStickX) / (denominator * slow);
        double frontRightPower = (rotY - rotX - rightStickX) / (denominator * slow);
        double backRightPower = (rotY + rotX - rightStickX) / (denominator * slow);

        this.fl.setPower(frontLeftPower);
        this.fr.setPower(frontRightPower);
        this.bl.setPower(backLeftPower);
        this.br.setPower(backRightPower);
    }
    public void getToAngle(double angle) {
        CustomPID angleControl = new CustomPID(this.anglePID);
        angleControl.setSetpoint(Math.toRadians(angle));
        double range = Math.PI / 90;
        if(!(Math.abs(imu.getAngularOrientation().firstAngle - Math.toRadians(angle)) <= range / 2.0)) {
            double[] results = angleControl.calculateGivenError(AngleWrap(Math.toRadians(angle) - imu.getAngularOrientation().firstAngle));
            this.fl.setPower(-results[0]);
            this.fr.setPower(results[0]);
            this.bl.setPower(-results[0]);
            this.br.setPower(results[0]);
        }
    }
    private  double AngleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2*Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public void moveInDirection(double theta, double power){
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);

        this.fl.setPower(power * cos);
        this.fr.setPower(power * sin);
        this.bl.setPower(power * sin);
        this.br.setPower(power * cos);
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public DcMotor getxOdom() {
        return xOdom;
    }

    public DcMotor getyOdom() {
        return yOdom;
    }

    /**
     * @param theta  The Angle that the robot should go(standard position)
     * @param distance  The distance that the robot should go
     */
    public void driveToLocation(double theta, double distance) {
        //Reset Encoders
        this.xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.xOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Initialize objects for robot tracking
        CustomPID xPID = new CustomPID(this.xPID);
        CustomPID yPID = new CustomPID(this.yPID);
        double targetX = distance * Math.cos(Math.toRadians(theta));
        double targetY = distance * Math.sin(Math.toRadians(theta));
        xPID.setSetpoint(targetX);
        yPID.setSetpoint(targetY);
        Vector motion = new Vector();
        double powerX = xPID.calculateGivenRaw(this.xOdom.getCurrentPosition())[0];
        double powerY = yPID.calculateGivenRaw( this.yOdom.getCurrentPosition())[0];
        ElapsedTime timer = new ElapsedTime();
        //while the x is out of range or y is out of range or the velocity is too fast
        while((Math.abs(powerX)>0.03||Math.abs(powerY)>0.03)&&timer.seconds()<5){
            //use the Pid to calculate the Powers
            powerX = xPID.calculateGivenRaw(this.xOdom.getCurrentPosition())[0];
            powerY = yPID.calculateGivenRaw( this.yOdom.getCurrentPosition())[0];
            //update the motion vector, run the move cmd
            motion.setComps(new double[]{powerX, powerY});
            moveInDirection(motion.theta(), motion.getMag());
        }
    }
    public void stayPut(double seconds, double angle) {
        //Reset Encoders
        this.xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.xOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Initialize objects for robot tracking
        CustomPID xPID = new CustomPID(this.xPID);
        CustomPID yPID = new CustomPID(this.yPID);
        xPID.setSetpoint(0);
        yPID.setSetpoint(0);
        Vector motion = new Vector();
        double powerX = xPID.calculateGivenRaw(this.xOdom.getCurrentPosition())[0];
        double powerY = yPID.calculateGivenRaw( this.yOdom.getCurrentPosition())[0];
        ElapsedTime timer = new ElapsedTime();
        //while the x is out of range or y is out of range or the velocity is too fast
        while(timer.seconds()<seconds){
            getToAngle(angle);
            //use the Pid to calculate the Powers
            powerX = xPID.calculateGivenRaw(this.xOdom.getCurrentPosition())[0];
            powerY = yPID.calculateGivenRaw( this.yOdom.getCurrentPosition())[0];
            //update the motion vector, run the move cmd
            motion.setComps(new double[]{powerX, powerY});
            moveInDirection(motion.theta(), motion.getMag());
        }
    }
    public void curveMove(double distance, double theta, double pocketDepth, double numSplits, double[] xPid, double[] yPid){
        double targetX = distance * Math.cos(Math.toRadians(theta));
        double targetY = distance * Math.sin(Math.toRadians(theta));
        ArrayList<Vector> pathList= new ArrayList<Vector>();
        pathList.add(new Vector(targetX, targetY));
        ArrayList<Vector> splits = new ArrayList<Vector>();
        for (int i = 0; i < numSplits; i++) {
            for (int j = 0; j < pathList.size(); j++) {
                Vector[] temps;
                if(i == 0){
                    temps = performSplit(pathList.get(j),pocketDepth);
                }else {
                    temps = performSplit(pathList.get(j), pocketDepth / (Math.pow(Math.E, 0.5 + i)));
                }
                splits.addAll(Arrays.asList(temps));
            }
            pathList.addAll(splits);
        }
        for(int i = 0; i < pathList.size(); i++){
            driveToLocation(pathList.get(i).theta(),pathList.get(i).getMag());
        }
    }


    private Vector[] performSplit(Vector path, double pocketDepth){
        double tangentTheta;
        if(pocketDepth>0){
            tangentTheta = path.theta()-Math.PI/2;
        }else {
            tangentTheta = path.theta() + Math.PI/2;
        }
        double tangentX = pocketDepth * Math.cos(tangentTheta);
        double tangentY = pocketDepth * Math.sin(tangentTheta);
        Vector tangent = new Vector(tangentX, tangentY);
        Vector half = new Vector(path.getComps()[0]/2,path.getComps()[1]/2);
        Vector vec1 = half.addition(tangent);
        Vector vec2 = path.subtraction(vec1);
        return new Vector[]{vec1, vec2};
    }
}
