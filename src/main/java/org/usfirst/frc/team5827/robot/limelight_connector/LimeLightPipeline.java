// package org.usfirst.frc.team5827.robot.limelight_connector;


// /**
//  * A pipeline handler.
//  */
// public enum LimeLightPipeline
// { 
//     CLOSEST(0);

//     // Initialize a part of the enumeration.
//     private LimeLightPipeline(int id)
//     {
//         identifier = id;
//     }

//     // Get the identification number
//     //of a pipeline.
//     public int getIdentifier()
//     {
//         return identifier;
//     }

//     private int identifier;

//     // Static methods:
//     public static LimeLightPipeline getCurrentPipe()
//     {
//         // Get the identifier of the pipe to use.
//         int pipeId = LimeLightConnector.getSelectedPipelineId();

//         LimeLightPipeline selectedPipe = null; // By default, no pipe was found.

//         // Search for that pipeline.
//         LimeLightPipeline[] pipelines = LimeLightPipeline.values();

//         // There should be no more than 10
//         //pipelines, so an efficient algorithm
//         //should not be necessary here.
//         for(LimeLightPipeline pipe : pipelines)
//         {
//             // If these have the same ID, assume they are the same.
//             if (pipe.getIdentifier() == pipeId)
//             {
//                 selectedPipe = pipe;

//                 break;
//             }
//         }

//         // Return the selected pipe or null.
//         return selectedPipe;
//     }
// }