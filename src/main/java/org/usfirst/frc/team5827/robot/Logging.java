package org.usfirst.frc.team5827.robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.TimeZone;
import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Formatter;
import java.util.logging.LogRecord;

import edu.wpi.first.wpilibj.DriverStation;

public class Logging
{
	/**
	 * Open print stream that writes to the log file. Example of use:
	 * exception.printStackTrace(Util.logPrintStream);
	 */
	public static final PrintStream logPrintStream = new PrintStream(new LoggingOutputStream());

	/**
	 * Logging class for use by other classes to log though this custom logging
	 * scheme. All logging should be done by calls to methods on this class instance
	 * or with the convenience methods of the Logging class.
	 */
	public final static Logger logger = Logger.getGlobal();

	public static BufferedWriter gyroWriter;

	public static BufferedWriter powerWriter;
	
	public static BufferedWriter operatorWriter;

	private static long initialMillis;

	// Private constructor means this class cannot be instantiated. All access is
	// static.

	private Logging()
	{
	}

	/**
	 * Configures and holds (static) classes for our custom logging system. Call
	 * setup() method to initialize logging.
	 */
	public static class CustomLogger
	{
		static private FileHandler fileTxt;

		static private LogFormatter logFormatter;

		/**
		 * Initializes our logging system. Call before using any logging methods.
		 */
		static public void setup() throws IOException
		{
			// get reference point for time values
			initialMillis = System.currentTimeMillis();

			// get the global logger to configure it and add a file handler.
			Logger logger = Logger.getGlobal();

			logger.setLevel(Level.ALL);

			// If we decide to redirect system.out to our log handler, then following
			// code will delete the default log handler for the console to prevent
			// a recursive loop. We would only redirect system.out if we only want to
			// log to the file. If we delete the console handler we can skip setting
			// the formatter...otherwise we set our formatter on the console logger.

			Logger rootLogger = Logger.getLogger("");

			Handler[] handlers = rootLogger.getHandlers();

			// if (handlers[0] instanceof ConsoleHandler)
			// {
			// rootLogger.removeHandler(handlers[0]);
			// return;
			// }

			logFormatter = new LogFormatter();

			// Set our formatter on the console log handler.
			if (handlers[0] instanceof ConsoleHandler)
				handlers[0].setFormatter(logFormatter);

			// Now create a handler to log to a file on roboRio "disk".

			// if (true) throw new IOException("Test Exception");
			
			//File location
			String basePath = "/home/lvuser/";
			String timeBase = new SimpleDateFormat("yyyy-MM-dd-HHmm.ss").format(new Date());
			
			fileTxt = new FileHandler(basePath + "Log" + timeBase + ".txt");
			fileTxt.setFormatter(logFormatter);

			logger.addHandler(fileTxt);

			// Now create the comma separated data files
			String gyroFileName = "Gyro-" + timeBase + ".csv";
			String powerFileName = "Power-" + timeBase + ".csv";
			String operatorFileName = "Operator-" + timeBase + ".csv";
			try
			{
				// Path path = FileSystems.getDefault().getPath(basePath, fileName);
				
				//Initialize writers
				gyroWriter = new BufferedWriter(new FileWriter(basePath + gyroFileName));
				powerWriter = new BufferedWriter(new FileWriter(basePath + powerFileName));
				operatorWriter = new BufferedWriter(new FileWriter(basePath + operatorFileName));
				
				//Create headers for logging files
				String gyroHeader = "Time, Yaw, Pitch, Roll\n";
				gyroWriter.write(gyroHeader, 0, gyroHeader.length());
				String powerHeader = "Time, Voltage, Current\n";
				powerWriter.write(powerHeader, 0, powerHeader.length());
				String operatorHeader = "Time, Drive, Turn, Lift, Intake, Outtake\n";
				operatorWriter.write(operatorHeader, 0, operatorHeader.length());
			}
			catch (IOException x)
			{
				System.err.format("IOException: %s%n", x);
			}
		}
	}

	// Our custom formatter for logging output.

	private static class LogFormatter extends Formatter
	{
		SimpleDateFormat dateFormat = new SimpleDateFormat("hh:mm:ss:S");

		public LogFormatter()
		{
			dateFormat.setTimeZone(TimeZone.getTimeZone("America/Los_Angeles"));
		}

		public String format(LogRecord rec)
		{
			StringBuffer buf = new StringBuffer(1024);

			buf.append(String.format("<%d>", rec.getThreadID()));
			buf.append(dateFormat.format(new Date(rec.getMillis())));
			buf.append(" ");
			buf.append(formatMessage(rec));
			buf.append("\n");

			return buf.toString();
		}
	}

	// An output stream that writes to our logging system. Writes data with flush on
	// flush call or on a newline character in the stream.

	private static class LoggingOutputStream extends OutputStream
	{
		private static final int DEFAULT_BUFFER_LENGTH = 2048;

		private boolean hasBeenClosed = false;

		private byte[] buf;

		private int count, curBufLength;

		public LoggingOutputStream()
		{
			curBufLength = DEFAULT_BUFFER_LENGTH;
			buf = new byte[curBufLength];
			count = 0;
		}

		public void write(final int b) throws IOException
		{
			if (hasBeenClosed)
			{
				throw new IOException("The stream has been closed.");
			}

			// don't log nulls
			if (b == 0)
				return;

			// force flush on newline character, dropping the newline.
			if ((byte) b == '\n')
			{
				flush();
				return;
			}

			// would this be writing past the buffer?
			if (count == curBufLength)
			{
				// grow the buffer
				final int newBufLength = curBufLength + DEFAULT_BUFFER_LENGTH;
				final byte[] newBuf = new byte[newBufLength];
				System.arraycopy(buf, 0, newBuf, 0, curBufLength);
				buf = newBuf;
				curBufLength = newBufLength;
			}

			buf[count] = (byte) b;

			count++;
		}

		public void flush()
		{
			if (count == 0)
				return;

			final byte[] bytes = new byte[count];

			System.arraycopy(buf, 0, bytes, 0, count);

			String str = new String(bytes);

			consoleLog(str);

			count = 0;
		}

		public void close()
		{
			flush();
			hasBeenClosed = true;
		}
	}

	/**
	 * Returns program location where call to this method is located.
	 */
	public static String currentMethod()
	{
		return currentMethod(2);
	}

	private static String currentMethod(Integer level)
	{
		StackTraceElement stackTrace[];

		stackTrace = new Throwable().getStackTrace();

		// This scheme depends on having one level in the package name between
		// team1111 and the class name, ie: team1111.robot.Logging.method. New levels
		// will require rewrite.

		try
		{
			String method = stackTrace[level].toString().split("5827.")[1];

			int startPos = method.indexOf(".") + 1;

			return method.substring(startPos);
		}
		catch (Throwable e)
		{
			return "method not found";
		}
	}

	/**
	 * Write warning message to console log with optional formatting and program
	 * location.
	 * 
	 * @param message
	 *            Message with optional format specifiers for listed parameters.
	 * @param parms
	 *            Parameter list matching format specifiers.
	 */
	public static void warningLog(String message, Object... parms)
	{
		// logs to the console as well as our log file on RR disk.
		logger.log(Level.WARNING, String.format("robot: %s: %s", currentMethod(2), String.format(message, parms)));
	}

	/**
	 * Write message to console log with optional formatting and program location.
	 * 
	 * @param message
	 *            Message with optional format specifiers for listed parameters.
	 * @param parms
	 *            Parameter list matching format specifiers.
	 */
	public static void consoleLog(String message, Object... parms)
	{
		// logs to the console as well as our log file on RR disk.
		logger.log(Level.INFO, String.format("robot: %s: %s", currentMethod(2), String.format(message, parms)));
	}

	/**
	 * Write blank line with program location to the console log.
	 */
	public static void consoleLog()
	{
		// logs to the console as well as our log file on RR disk.
		logger.log(Level.INFO, String.format("robot: %s", currentMethod(2)));
	}

	/**
	 * Writes a new line with current gyro data to logging file
	 * @param yaw The yaw value obtained from gyro
	 * @param pitch The pitch value obtained from gyro
	 * @param roll The roll value obtained from gyro
	 * @throws IOException
	 */
	public static void gyroData(double yaw, double pitch, double roll) throws IOException
	{
		// String timeString = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.S").format(new
		// Date());
		long timeString = System.currentTimeMillis() - initialMillis;
		String output = String.format("%s, %f, %f, %f\n", timeString, yaw, pitch, roll);
		gyroWriter.write(output, 0, output.length());
		gyroWriter.flush();

	}
	
	/**
	 * Writes a new line with current voltage and current data to logging file
	 * @param voltage Input voltage of the PDP
	 * @param current Total current of PDP channels
	 * @throws IOException
	 */
	public static void powerData(double voltage, double current) throws IOException
	{
		// String timeString = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.S").format(new
		// Date());
		long timeString = System.currentTimeMillis() - initialMillis;
		String output = String.format("%s, %f, %f\n", timeString, voltage, current);
		powerWriter.write(output, 0, output.length());
		powerWriter.flush();
	}
	
	/**
	 * Writes a new line with current operator input to logging file
	 * @param driveAxis Forward and backward motion
	 * @param turnAxis Left and right turning
	 * @param liftAxis Lifter operation
	 * @param cubeIntake Whether cube intake button is pressed
	 * @param cubeOuttake Whether cube outtake button is pressed
	 * @throws IOException
	 */
	public static void operatorData(double driveAxis, double turnAxis, double liftAxis,
			boolean cubeIntake, boolean cubeOuttake) throws IOException
	{
		// String timeString = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.S").format(new
		// Date());
		long timeString = System.currentTimeMillis() - initialMillis;
		
		String output = String.format("%s, %f, %f, %f, %d, %d\n", timeString,
				driveAxis, turnAxis, liftAxis,
				((cubeIntake)? 1:0), ((cubeOuttake)? 1:0));
		
		operatorWriter.write(output, 0, output.length());
		operatorWriter.flush();
	}

	/**
	 * Write exception message to DS console window and exception stack trace to log
	 * file.
	 * 
	 * @param e
	 *            The exception to log.
	 */
	public static void logException(Throwable e)
	{
		DriverStation.reportError(e.toString(), false);

		e.printStackTrace(Logging.logPrintStream);
	}
}