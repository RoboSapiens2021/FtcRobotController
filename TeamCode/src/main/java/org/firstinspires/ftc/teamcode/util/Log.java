package org.firstinspires.ftc.teamcode.util;

public interface Log {
    enum LEVEL {
        INFO, WARN, ERROR, DEBUG, TRACE
    }

    boolean isInfoEnabled();

    boolean isDebugEnabled();

    boolean isErrorEnabled();

    boolean isWarnEnabled();

    boolean isTraceEnabled();

    void info(Object message);

    void error(Object message);

    void debug(Object message);

    void warn(Object message);

    void trace(Object message);

}
