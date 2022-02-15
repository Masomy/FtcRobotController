package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class BaseComponent implements Component {

    private OpMode opMode;

    protected HardwareMap hardwareMap;

    protected Telemetry telemetry;

    protected ElapsedTime time;

    private Command currentCommand;

    private List<Component> subComponents = new ArrayList<>();

    public BaseComponent(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.time = new ElapsedTime();
        this.currentCommand = null;
    }

    protected void executeCommand(Command command) {
        stopCommand();
        time.reset();
        currentCommand = command;
        currentCommand.start();
    }

    protected void stopCommand() {
        if(currentCommand != null) {
            currentCommand.stop();
            currentCommand = null;
        }
    }

    protected void addSubComponents(Component... subComponents) {
        this.subComponents.addAll(Arrays.asList(subComponents));
    }

    @Override
    public void init() {
        for (Component subComponent : subComponents) {
            telemetry.addData("Subcomponent:",subComponent);
            telemetry.update();
            subComponent.init();
        }
        telemetry.addData("Robot is initialized","");
        telemetry.update();
    }

    @Override
    public void updateStatus() {
        // If there is a current command we are trying to execute, delegate to it to update status
        if (currentCommand != null) {
            boolean finished = currentCommand.updateStatus();

            // If the command is finished, remove it
            if (finished) {
                currentCommand.stop();
                currentCommand = null;
            }
        }

        // Also update any sub-components
        for (Component subComponent : subComponents) {
            subComponent.updateStatus();
        }
    }

    @Override
    public boolean isBusy() {
        // We are busy if any child component is busy.
        for (Component subComponent : subComponents) {
            if (subComponent.isBusy()) {
                return true;
            }
        }

        // We are busy if we have a command we are trying to execute and that command is still busy.
        return currentCommand != null;
    }

    protected boolean isStopRequested() {
        return opMode instanceof LinearOpMode && ((LinearOpMode) opMode).isStopRequested();
    }

    protected void sleep(long millis) {
        if (opMode instanceof LinearOpMode) {
            LinearOpMode opMode = (LinearOpMode) this.opMode;
            opMode.sleep(millis);
            opMode.idle();
        }
    }

}
