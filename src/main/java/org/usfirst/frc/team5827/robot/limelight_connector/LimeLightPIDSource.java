// package org.usfirst.frc.team5827.robot.limelight_connector;

// import edu.wpi.first.wpilibj.PIDSource;
// import edu.wpi.first.wpilibj.PIDSourceType;

// /**
//  * A PID Source.
//  */
// public class LimeLightPIDSource implements PIDSource
// {
//     private PIDSourceType sourceType;
//     private LimeLightConnector.AccessibleProperties willAccess;

//     public LimeLightPIDSource(LimeLightConnector.AccessibleProperties propertyToAccess)
//     {
//         willAccess = propertyToAccess;
//         sourceType = PIDSourceType.kDisplacement;
//     }

//     public void setPIDSourceType(PIDSourceType type)
//     {
//         sourceType = type;
//     }

//     public PIDSourceType getPIDSourceType()
//     {
//         return sourceType;
//     }

//     public double pidGet()
//     {
//         return LimeLightConnector.getGeneralProperty(willAccess);
//     }
// }