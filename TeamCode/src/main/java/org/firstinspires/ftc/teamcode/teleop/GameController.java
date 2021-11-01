package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.motor.Spinner;
import org.firstinspires.ftc.teamcode.util.Logger;

public class GameController {
    private Logger logger = Logger.getInstance();

    private Spinner spinner;

    public GameController(Spinner spinner) {
        this.spinner = spinner;
    }

    public void turnSpinner(double power) {
        spinner.spin(power);
    }

}
