package org.firstinspires.ftc.teamcode.hardware.subsytems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Constraints;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Config
@TeleOp(name = "ElevatorPID")
public class ElevatorSubsystem implements SubsystemBase {
    /// definindo os motores do Elevador ///
    RobotHardware Robot;
    int elevatorPosition;
    double pidPower = 0;
    private PIDController controller;
    Constraints.ElevatorConstraints constraints = new Constraints.ElevatorConstraints();

    public static int target = 0;

    public ElevatorSubsystem(RobotHardware robot){
        this.Robot = robot;
    }
    public void init() {
        controller = new PIDController(constraints.kp, constraints.ki, constraints.kd);

        int elevatorPosition = Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition();


    }
    public void periodic(){
        elevatorPosition = -((Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition())/10);
    }

    public void manualControl(float upButton, float downButton) {
        Robot.LSi.setPower(upButton - downButton);
        Robot.LSii.setPower(upButton - downButton);
    }

    public void pidManualControl(float upButton, float downButton){
        double position;
        int lastPosition = 0;
        Robot.LSi.setPower(upButton - downButton * 0.7);
        Robot.LSii.setPower(upButton - downButton * 0.7);
        elevatorPosition = (Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition());
        if (Robot.LSi.getPower() == 0 && Robot.LSii.getPower() == 0){
            pidTarget(lastPosition);
        }
        else{
            lastPosition = elevatorPosition;
        }
    }
    public void pidTarget(int target){
        elevatorPosition = (Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition());
        pidPower = controller.calculate(elevatorPosition,target);
        Robot.LSi.setPower(pidPower);
        Robot.LSii.setPower(pidPower);
    }

}
