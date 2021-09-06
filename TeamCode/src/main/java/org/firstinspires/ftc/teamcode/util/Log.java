package org.firstinspires.ftc.teamcode.util;

public interface Log {
    enum CAPTION {
        Status, Position
    }

    void log(Logger.CAPTION caption, String format, Object... args);

    void log(Logger.CAPTION caption, Object message);

    void log(String format, Object... args);

    void log(Object message);

}
