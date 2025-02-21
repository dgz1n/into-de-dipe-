package org.firstinspires.ftc.teamcode.hardware.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Disabled
public class ClawSubystem implements SubsystemBase {
    private RobotHardware Robot;
    boolean wristIsTurned, clawIsOpen;
    ElapsedTime timer = new ElapsedTime();
    public ClawSubystem(RobotHardware Robot){
        this.Robot = Robot;
    }
    public void init(){
        wristIsTurned = false;
        clawIsOpen = true;
    }
    public void periodic() {

    }
    public void manualControl(boolean wrist, boolean claw){
        if (wrist && timer.seconds() >= 0.3){
            if (!wristIsTurned){
                Robot.wristServo.setPosition(0.37);
                wristIsTurned = true;
            }
            else if (wristIsTurned == true){
                Robot.wristServo.setPosition(0);
                wristIsTurned = false;
            }
            timer.reset();
        }

        //Abrir/fechar a garra
        if (claw && timer.seconds() >= 0.5){
            if (clawIsOpen == true){
                Robot.clawServo.setPosition(0.15);
                clawIsOpen = false;
            }
            else if (!clawIsOpen) {
                Robot.clawServo.setPosition(0);
                clawIsOpen = true;
            }
            timer.reset();
        }
    }
}
