package org.usfirst.frc.team5827.robot;

public class MathHelper 
{
	/**
	 *  Parse an integer, without errors.
	 * @param stringToConvert A string, null or non-null to be converted to an integer.
	 * @return The given string, as an integer.
	 */
	public static int forceParseInt(String stringToConvert)
	{
		if(stringToConvert == null) // Allow input to be null.
		{
			return 0;
		}
		
		int result = 0, // Store the result here.
				power = 0; // The power to raise 10 to.
		
		for(int characterIndex = stringToConvert.length() - 1; characterIndex >= 0; characterIndex--)
		{
			// Add the digit to the result.
			result += ((int)(stringToConvert.charAt(characterIndex) - '0')) * Math.pow(10, power);
			
			power++;
		}
		
		return result;
	}
	
	/**
	 *  Like forceParseInt, but returns a double.
	 *  This function operates in base 10.
	 * @param stringToConvert A string to convert to a double.
	 * @return The double version of the given string.
	 */
	public static double forceParseDouble(String stringToConvert)
	{
		if(stringToConvert == null) // Give an output, even if input is null.
		{
			return 0.0;
		}
		
		int decimalLocation = stringToConvert.indexOf('.'); // Find the location of the decimal point.
		
		double result = 0;  // Output will be stored in result.
		int power = 0; // 10 is raised to powers of an integer.
		
		if(decimalLocation == -1) // If no decimal is found,
		{
			power = 0;
		}
		else
		{
			power = decimalLocation - stringToConvert.length() + 1; // The number of digits after the decimal point.
		}
		
		char currentChar;
		
		// Loop from the last character to the first.
		for(int characterIndex = stringToConvert.length() - 1; characterIndex >= 0; characterIndex--)
		{
			currentChar = stringToConvert.charAt(characterIndex);
			
			if(currentChar <= '9' && currentChar >= '0')
			{
				result += ((double)(currentChar - '0')) * Math.pow(10, power);
				
				// Only increase the power if a digit was added to the result.
				power++;
			}
		} // End for.
		
		return result;
	}

	/**
	 * Return the distance between the two provided points.
	 * @param a One of the two points.
	 * @param b The other.
	 * @return Returns the distance (calculated using the 
	 * Pythagorean theorem) between a and b.
	 */
	public static double getDistance(Point2D a, Point2D b)
	{
		double deltaX = a.getX() - b.getX(),
				deltaY = a.getY() - b.getY();

		double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
		
		return distance;
	}

	/**
	 * Returns the distance give a and b
	 * where:
	 *     /|
	 *    / |
	 * d /  | b
	 *  /   |
	 * /    |
	 * ------
	 *    a
	 * 
	 * @return d
	 */
	public static double getDistance(double a, double b)
	{
		double distance = Math.sqrt(a*a + b * b);

		return distance;
	}

	/**
	 * Manage 2-dimensional points.
	 */
	public static class Point2D
	{
		private double m_x, m_y;

		public Point2D(double x, double y)
		{
			m_x = x;
			m_y = y;
		}

		public double getX()
		{
			return m_x;
		}

		public double getY()
		{
			return m_y;
		}

		public void setX(double newX)
		{
			m_x = newX;
		}

		public void setY(double newY)
		{
			m_y = newY;
		}

		public String toString()
		{
			return "{" + Double.toString(m_x) + ", " + Double.toString(m_y) + "}";
		}
	}

	/**
	 * To provide readability for line collision math.
	 */
	public static class Line2D
	{
		private Point2D m_point1, m_point2;

		public Line2D(Point2D point1, Point2D point2)
		{
			m_point1 = point1;
			m_point2 = point2;
		}

		public Line2D(double dx, double dy, Point2D pointOnLine)
		{
			m_point1 = pointOnLine;
			m_point2 = new Point2D(m_point1.getX() + dx, m_point1.getY() + dy);
		}

		// Return the change in x, used for slope calculations.
		public double getDx()
		{
			return m_point1.getX() - m_point2.getX();
		}

		// Return the change in y, used for slope calculations.
		public double getDy()
		{
			return m_point1.getY() - m_point2.getY();
		}

		// Returns delta y over delta x.
		public double getSlope()
		{
			double deltaY, deltaX, slope = 0;

			deltaY = m_point1.getY() - m_point2.getY();
			deltaX = m_point1.getX() - m_point2.getX();

			if(deltaX != 0.0)
			{
				slope = deltaY / deltaX;
			}
			else
			{
				slope = 0.00001; // Return a very small, approximate slope.
			}

			return slope;
		}

		// Returns a line perpindicular to this one through a point.
		public Line2D getPerpindicularAt(Point2D at)
		{
			Line2D result;

			double slope = -1 / getSlope();
			result = new Line2D(1, slope, at);

			return result;
		}

		// Returns where two lines collide.
		public Point2D getCollision(Line2D other)
		{
			Point2D result = new Point2D(0, 0);
			double b = getYIntercept().getY(),
				b2 = other.getYIntercept().getY();
			double mySlope = getSlope(),
				otherSlope = other.getSlope();

			result.setX((b - b2) / (otherSlope - mySlope));
			result.setY(mySlope * result.getX() + b);

			return result;
		}

		// Get the line's y-intercept.
		public Point2D getYIntercept()
		{
			double slope = getSlope();
			return new Point2D(0, -slope * m_point1.getX() + m_point1.getY());
		}
	}
}