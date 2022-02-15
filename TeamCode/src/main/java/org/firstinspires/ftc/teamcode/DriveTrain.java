package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.DistanceUtil.inches;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.PointUtil;
import org.opencv.core.Point;

import java.util.Arrays;
import java.util.List;

public class DriveTrain extends BaseComponent {

    // todo: edge detection (can't rely solely on runToPosition)

    /**
     * Diameter of the wheel in feet
     */
    private static final double WHEEL_SIZE = 0.328084;

    private static final double TICKS_PER_REVOLUTION = 537.6;
    private static final double STRAFE_CORRECTION = 1.25;

    /**
     * The software of the drivetrain
     */
    private TileEdgeDetector tileEdgeDetectorSide;
    private TileEdgeDetector tileEdgeDetectorRear;

    /**
     * The hardware for the drive train
     */
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    /**
     * Where we think the robot is heading.  (0 to 360)
     */
    private double heading;

    /**
     * Where we think the robot is (measured from the starting point at init time).
     */
    private Point position;

    /**
     * The last orientation obtained from the IMU.
     */
    private Orientation lastOrientation;


    public DriveTrain(OpMode opMode, WebCam webCamRear, WebCam webCamSide) {
        super(opMode);

        frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FrontLeft");
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("FrontRight");
        backLeft = (DcMotorEx) hardwareMap.dcMotor.get("BackLeft");
        backRight = (DcMotorEx) hardwareMap.dcMotor.get("BackRight");
        motors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        tileEdgeDetectorSide = new TileEdgeDetector(opMode, webCamSide);
        tileEdgeDetectorRear = new TileEdgeDetector(opMode, webCamRear);
        addSubComponents(tileEdgeDetectorSide);
        addSubComponents(tileEdgeDetectorRear);

        position = new Point(0, 0);
        heading = 90.0;
    }

    @Override
    public void init() {
        super.init();

        initIMU();

        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection((DcMotorSimple.Direction.FORWARD));

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Activate the side tile edge detector immediately
        tileEdgeDetectorSide.activate();
    }

    public void activateRearTileEdgeDetection() {
        // Activate rear tile edge detection after the rear webcam is available
        tileEdgeDetectorRear.activate();
    }

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        telemetry.addData("IMU", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
        }

        telemetry.addData("IMU", "waiting for start...");
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Start integration background thread, so we can get updated position in a loop.
        //imu.startAccelerationIntegration(null, null, 5);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void updateStatus() {
        // Update the heading based off the IMU
        updateHeading();
        updatePosition();

        // todo: Update the position based off of encoders

        telemetry.addData("Heading", heading);
        telemetry.addData("Position", position);

        if (tileEdgeDetectorSide.isDetected()) {
            telemetry.addData("Angle to Tile", String.format("%.2f", tileEdgeDetectorSide.getAngleToTile()));
            telemetry.addData("Distance to Tile", String.format("%.2f in", tileEdgeDetectorSide.getDistanceToTile() * 12.0));
        }

        // Now allow any commands to run with the updated data
        super.updateStatus();
    }

    /**
     * Updates the heading with the IMU
     */
    private void updateHeading() {

        // The following code adapted with permission from team SkyStone 2019-2020.

        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (lastOrientation == null) {
            lastOrientation = orientation;
        }

        double deltaAngle = orientation.firstAngle - lastOrientation.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        heading += deltaAngle;

        lastOrientation = orientation;
    }

    /**
     * Updates the current position using observed data (e.g. IMU and tile edge detection)
     */
    private void updatePosition() {

        // Get the current position from the IMU in feet
        Position position = imu.getPosition();
        double x = inches(position.unit.toInches(position.x));
        double y = inches(position.unit.toInches(position.y));

        if (position.acquisitionTime != 0) {

            // Get the velocity in feet / second
            Velocity velocity = imu.getVelocity();
            double vx = inches(velocity.unit.toInches(velocity.xVeloc));
            double vy = inches(velocity.unit.toInches(velocity.yVeloc));

            // Assuming we continued at the same velocity since the acquisition time, update the
            // position based on the age of the data.
            double elapsed = (System.nanoTime() - position.acquisitionTime) / 1e9;
            x += vx * elapsed;
            y += vy * elapsed;

        }

        // todo: add tile edge detection here

        this.position = new Point(x, y);
    }

    /**
     * Moves forward the given distance.
     *
     * @param distance the distance to move in feet
     * @param speed    a factor 0-1 that indicates how fast to move
     */
    public void moveForward(double distance, double speed) {
        executeCommand(new MoveForward(distance, speed));
    }

    public void moveForwardByObservedPosition(double distance, double speed) {
        // todo: this is a temporary command for testing movement based on an observed position
        //  rather than motor encoder ticks (e.g. IMU or tile edge detector)
        executeCommand(new MoveForwardByObservedPosition(distance, speed));
    }

    /**
     * Moves backward the given distance.
     *
     * @param distance the distance to move in feet
     * @param speed    a factor 0-1 that indicates how fast to move
     */
    public void moveBackward(double distance, double speed) {
        executeCommand(new MoveForward(-distance, speed));
    }

    /**
     * Moves forward the given distance
     *
     * @param distance  the distance to move in feet
     * @param speed     a factor 0-1 that indicates how fast to move
     * @param direction the direction we want to travel
     */
    public void strafe(double distance, double speed, StrafeDirection direction) {
        executeCommand(new Strafe(distance, speed, direction));
    }

    public void strafeWithoutRamping(double time, double speed, StrafeDirection direction) {
        executeCommand(new StrafeWithoutRamping(time,speed,direction));
    }

    /**
     * Turns the given angle
     *
     * @param angle Positive is left, negative is right, turns the given angle in degrees
     * @param speed 0-1, how fast we move
     */
    public void rotate(double angle, double speed) {
        executeCommand(new Rotate(angle, speed));
    }

    /**
     * Moves the robot to a point relative to it
     *
     * @param destination the point relative to the robot we are approaching. The points are feet.
     * @param speed       the speed at which to go
     */
    public void vectorMove(Point destination, double speed) {
        executeCommand(new VectorMove(destination, speed));
    }

    /**
     * Aligns the robot to the given angle from the edge of the tile.
     *
     * @param targetAngle the desired angle from the tile edge, in degrees.
     * @param speed       the master speed at which we travel
     * @param time        the time the detector will wait in seconds
     */
    public void alignToTileAngle(double targetAngle, double speed, double time) {
        if (tileEdgeDetectorSide.waitForDetection(time)) {
            double initialAngle = tileEdgeDetectorSide.getAngleToTile();
            double angle = targetAngle - initialAngle;

            double maxRotationForAlignment = 45.0;
            if (Math.abs(angle) < maxRotationForAlignment) {
                rotate(angle, speed);
            }
        }
    }

    public void alignToTileAngle(double targetAngle, double speed) {
        alignToTileAngle(targetAngle,speed,1);
    }

    /**
     * Strafes the robot so that the edge of the right wheel is the requested distance from the edge of the closest
     * tile to the right.
     *
     * @param targetDistance the desired distance from the edge of the tile, in feet.
     * @param speed       the master speed at which we travel
     * @param time        the time the detector will wait in seconds
     */
    public void moveDistanceFromTileEdge(double targetDistance, double speed, double time) {
        if (tileEdgeDetectorSide.waitForDetection(time)) {
            double initialDistance = tileEdgeDetectorSide.getDistanceToTile();
            double distance = targetDistance - initialDistance;

            // Sanity check - don't try to move more than 6 inches
            double maxDistance = inches(10);
            if (Math.abs(distance) < maxDistance) {
                StrafeDirection direction = distance < 0.0 ? StrafeDirection.RIGHT : StrafeDirection.LEFT;
                strafe(Math.abs(distance), speed, direction);
            }
        }
    }

    public void moveDistanceFromTileEdge(double targetDistance, double speed) {
        moveDistanceFromTileEdge(targetDistance,speed,1);
    }

    /**
     * Turns off all the motors.
     */
    private void stopMotors() {
        // Shut off the motor power
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    /**
     * Turns on all the motors.
     *
     * @param speed how fast the motors turn
     */
    private void setMotorPower(double speed) {
        // Shut off the motor power
        for (DcMotorEx motor : motors) {
            motor.setPower(speed);
        }
    }

    /**
     * Set the run mode for all motors.
     */
    private void setMotorMode(DcMotor.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    /**
     * Calculates the average position for each motor.
     */
    private int averageMotorPosition() {
        int sum = 0;
        for (DcMotorEx motor : motors) {
            sum += Math.abs(motor.getCurrentPosition());
        }
        return sum / motors.size();
    }

    /**
     * Runs each motor at the given speed for the given time in milliseconds.
     */
    public void runEachMotor(double speed, int time) {
        for (DcMotorEx motor : motors) {
            motor.setPower(speed);
            sleep(time);
        }
    }

    /**
     * Converts feet traveled into number of ticks moved by
     *
     * @param distance how far you want to travel in cm
     * @return number of ticks to turn
     */
    private int feetToTicks(double distance) {
        //in feet
        double wheelCircumference = (WHEEL_SIZE * Math.PI);

        double wheelRevolutions = distance / wheelCircumference;

        //turns wheelRevolutions into degrees, then divides by DEGREES_PER_TICK
        return (int) Math.round(wheelRevolutions * TICKS_PER_REVOLUTION);
    }

    /**
     * Calculates a smooth power curve between any two positions (in ticks, degrees, inches, etc),
     * based on the current position, the initial position, and the target position.
     *
     * @param current the current measured position
     * @param initial the initial position that was moved from
     * @param target  the target position being moved to
     * @param speed   the master speed, with range 0.0 - 1.0
     */
    private double getPowerCurveForPosition(double current, double initial, double target, double speed) {
        // Avoid division by zero
        if (target == initial) return 0.0;

        // Scale the position to between 0 - 1
        double xVal = (current - initial) / (target - initial);

        double minPower = 0.2;

        double power;
        if (xVal < 0.25) {
            power = 1 / (1 + Math.pow(Math.E, -16 * (2 * xVal - 0.125)));

            // While accelerating, gradually increase the min power with time
            minPower += time.seconds() / 4.0;

        } else {
            power = 1 / (1 + Math.pow(Math.E, 8 * (2 * xVal - 1.675)));
        }

        power *= speed;

        if (power < minPower) {
            power = minPower;
        }

        return power;
    }

    private abstract class BaseCommand implements Command {

        @Override
        public void stop() {
            stopMotors();
        }
    }

    private class MoveForward extends BaseCommand {

        /**
         * The distance we want to move.
         */
        private double distance;

        /**
         * The speed at which to move (0 - 1).
         */
        private double speed;

        /**
         * The number of ticks we want to move.
         */
        private int ticks;

        public MoveForward(double distance, double speed) {
            this.distance = distance;
            this.speed = speed;
        }

        @Override
        public void start() {

            // Figure out the distance in ticks
            ticks = feetToTicks(distance);

            setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            for (DcMotorEx motor : motors) {
                motor.setTargetPosition(ticks);
            }

            setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }

        @Override
        public boolean updateStatus() {

            // Check if we've reached the correct number of ticks
            int ticksMoved = averageMotorPosition();

            double power = getPowerCurveForPosition(ticksMoved, 0, ticks, speed);

            telemetry.addData("ticks moved", ticksMoved);
            telemetry.addData("ticks", ticks);
            telemetry.addData("currentPower", power);

            setMotorPower(power);
            return ticksMoved >= Math.abs(ticks);
        }

    }

    private class MoveForwardByObservedPosition extends BaseCommand {

        /**
         * The distance we want to move.
         */
        private double distance;

        /**
         * The speed at which to move (0 - 1).
         */
        private double speed;

        /**
         * The origin point.
         */
        private Point origin;

        public MoveForwardByObservedPosition(double distance, double speed) {
            this.distance = distance;
            this.speed = speed;
        }

        @Override
        public void start() {

            // Remember the origin point
            this.origin = position;

            setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public boolean updateStatus() {

            // Check the distance from the destination point
            double distanceMoved = PointUtil.distance(position, origin);

            double power = getPowerCurveForPosition(distanceMoved, 0, distance, speed);

            telemetry.addData("Distance Moved", distanceMoved);
            telemetry.addData("Current Power", power);

            setMotorPower(power);
            return distanceMoved >= distance;
        }

    }

    public enum StrafeDirection {
        RIGHT,
        LEFT
    }

    private class Strafe extends BaseCommand {

        /**
         * The direction in which to strafe.
         */
        private StrafeDirection direction;

        /**
         * The distance we want to move.
         */
        private double distance;

        /**
         * The speed at which to move.
         */
        private double speed;

        private int ticks;

        // todo check distance multiplied by strafe modifier
        public Strafe(double distance, double speed, StrafeDirection direction) {
            this.direction = direction;
            this.distance = distance;
            this.speed = speed;
        }

        @Override
        public void start() {
            ticks = feetToTicks(distance);

            setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            if (direction == StrafeDirection.LEFT) {
                frontLeft.setTargetPosition(-ticks);
                frontRight.setTargetPosition(ticks);
                backLeft.setTargetPosition(ticks);
                backRight.setTargetPosition(-ticks);

            } else if (direction == StrafeDirection.RIGHT) {
                frontLeft.setTargetPosition(ticks);
                frontRight.setTargetPosition(-ticks);
                backLeft.setTargetPosition(-ticks);
                backRight.setTargetPosition(ticks);
            }

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        @Override
        public boolean updateStatus() {
            int ticksMoved = averageMotorPosition();

            telemetry.addData("tick moved", ticksMoved);
            telemetry.addData("ticks", ticks);

            setMotorPower(getPowerCurveForPosition(ticksMoved, 0, ticks, speed));
            return ticksMoved >= ticks;
        }
    }

    private class StrafeWithoutRamping extends BaseCommand {

        /**
         * The direction in which to strafe.
         */
        private StrafeDirection direction;

        /**
         * The distance we want to move.
         */
        private double duration;

        /**
         * The speed at which to move.
         */
        private double speed;

        // todo check distance multiplied by strafe modifier
        public StrafeWithoutRamping(double duration, double speed, StrafeDirection direction) {
            this.direction = direction;
            this.duration = duration;
            this.speed = speed;
        }

        @Override
        public void start() {
            setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            if (direction == StrafeDirection.LEFT) {
                frontLeft.setPower(-speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(-speed);

            } else if (direction == StrafeDirection.RIGHT) {
                frontLeft.setPower(speed);
                frontRight.setPower(-speed);
                backLeft.setPower(-speed);
                backRight.setPower(speed);
            }
        }

        @Override
        public boolean updateStatus() {
            return time.seconds() >= duration;
        }
    }


    private class Rotate extends BaseCommand {

        // The angle threshold in degrees
        private double ANGLE_THRESHOLD = 1.0;

        private double initialHeading;
        private double targetHeading;
        private double speed;

        public Rotate(double angle, double speed) {
            this.initialHeading = heading;
            this.targetHeading = heading + angle;
            this.speed = speed;
        }

        @Override
        public void start() {
            setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public boolean updateStatus() {
            double power = getPowerCurveForPosition(heading, initialHeading, targetHeading, speed);

            if (targetHeading < heading) {
                power = -power;
            }

            telemetry.addData("Heading", heading);
            telemetry.addData("Initial Heading", initialHeading);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Motor Power Curve", power);

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            return Math.abs(heading - targetHeading) < ANGLE_THRESHOLD;
        }


    }

    private class VectorMove extends BaseCommand {

        private double rightSlantDist;
        private double leftSlantDist;
        private double rightSlantSpeed;
        private double leftSlantSpeed;
        double speed;

        public VectorMove(Point point, double speed) {
            this.speed = speed;
            double tickY = feetToTicks(point.y);
            double tickX = feetToTicks(point.x);
            rightSlantDist = tickY - tickX;
            leftSlantDist = tickY + tickX;
        }

        @Override
        public void start() {
            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            double[] speedList = vectorMovementSpeedCalculator(leftSlantDist, rightSlantDist);
            leftSlantSpeed = speedList[0];
            rightSlantSpeed = speedList[1];

            backLeft.setTargetPosition((int) rightSlantDist);
            frontLeft.setTargetPosition((int) leftSlantDist);
            frontRight.setTargetPosition((int) rightSlantDist);
            backRight.setTargetPosition((int) leftSlantDist);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            double power = getPowerCurveForPosition(frontRight.getCurrentPosition(), 0, rightSlantDist, speed);

            backLeft.setPower(power * rightSlantSpeed);
            frontLeft.setPower(power * leftSlantSpeed);
            frontRight.setPower(power * rightSlantSpeed);
            backRight.setPower(power * leftSlantSpeed);
        }

        @Override
        public boolean updateStatus() {
            double power = getPowerCurveForPosition(backLeft.getCurrentPosition(), 0, rightSlantDist, speed);

            backLeft.setPower(power * rightSlantSpeed);
            frontLeft.setPower(power * leftSlantSpeed);
            frontRight.setPower(power * rightSlantSpeed);
            backRight.setPower(power * leftSlantSpeed);

            return vectorMoveCompletion(frontRight.getCurrentPosition(), frontLeft.getCurrentPosition(), rightSlantDist, leftSlantDist);
        }

        /**
         * Checks if the right and left slants are done with their movement
         *
         * @param rightCurrent right slants current position
         * @param leftCurrent  left slants current position
         * @param rightTarget  rigt slants target position
         * @param leftTarget   left slants target position
         * @return If the left and right slant are at their target positions.
         */
        private boolean vectorMoveCompletion(double rightCurrent, double leftCurrent, double rightTarget, double leftTarget) {
            return rightCurrent >= rightTarget && leftCurrent >= leftTarget;
        }

        /**
         * gets the speed for vector movement
         *
         * @param rightSlantDist
         * @param leftSlantDist
         * @return Array of the speeds needed. Index 0 is the left, Index 1 is the right
         */
        private double[] vectorMovementSpeedCalculator(double leftSlantDist, double rightSlantDist) {
            double[] speed = new double[2];
            if (rightSlantDist > leftSlantDist) {
                speed[0] = leftSlantDist / rightSlantDist;
                speed[1] = 1;
            } else if (rightSlantDist < leftSlantDist) {
                speed[0] = 1;
                speed[1] = rightSlantDist / leftSlantDist;
            } else {
                speed[0] = 1;
                speed[1] = 1;
            }
            return speed;
        }
    }

}
