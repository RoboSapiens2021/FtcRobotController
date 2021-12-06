package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logger implements Log {
    private static final Logger instance = new Logger();
    private Telemetry telemetry = null;

    private boolean errorEnabled = true;
    private boolean warnEnabled = true;
    private boolean infoEnabled = true;
    private boolean debugEnabled = true;
    private boolean traceEnabled = true;

    private Logger() {
        if (LEVEL.ERROR == Constants.logLevel) {
            warnEnabled = false;
            infoEnabled = false;
            debugEnabled = false;
            traceEnabled = false;
        } else if (LEVEL.WARN == Constants.logLevel) {
            infoEnabled = false;
            debugEnabled = false;
            traceEnabled = false;
        } else if (LEVEL.INFO == Constants.logLevel) {
            debugEnabled = false;
            traceEnabled = false;
        } else if (LEVEL.DEBUG == Constants.logLevel) {
            traceEnabled = false;
        }
    }

    public static Logger getInstance() {
        return instance;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public boolean isInfoEnabled() {
        return infoEnabled;
    }

    @Override
    public boolean isDebugEnabled() {
        return debugEnabled;
    }

    @Override
    public boolean isErrorEnabled() {
        return errorEnabled;
    }

    @Override
    public boolean isWarnEnabled() {
        return warnEnabled;
    }

    @Override
    public boolean isTraceEnabled() {
        return traceEnabled;
    }

    @Override
    public void info(Object message) {
        if (this.isInfoEnabled()) {
            innerLog(message);
        }
    }

    @Override
    public void error(Object message) {
        if (this.isErrorEnabled()) {
            innerLog(message);
        }
    }

    @Override
    public void warn(Object message) {
        if (this.isWarnEnabled()) {
            innerLog(message);
        }
    }

    @Override
    public void debug(Object message) {
        if (this.isDebugEnabled()) {
            innerLog(message);
        }
    }

    @Override
    public void trace(Object message) {
        if (this.isTraceEnabled()) {
            innerLog(message);
        }
    }

    private void innerLog(Object message) {
        if (null == telemetry) {
            System.out.println(message);
        } else {
            telemetry.addLine(message.toString());
            telemetry.update();
        }
    }

//    public void log(LEVEL LEVEL, String format, Object... args) {
//        if (null == telemetry) {
//            System.out.println(String.format(format, args));
//        } else {
//            telemetry.addData(LEVEL.name(), format, args);
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void log(LEVEL LEVEL, Object message) {
//        if (null == telemetry) {
//            System.out.println(message);
//        } else {
//            telemetry.addData(LEVEL.name(), message);
//            telemetry.update();
//        }
//    }
//
//    public void log(String format, Object... args) {
//        if (null == telemetry) {
//            System.out.println(String.format(format, args));
//        } else {
//            telemetry.addLine(String.format(format, args));
//            telemetry.update();
//        }
//    }


}
