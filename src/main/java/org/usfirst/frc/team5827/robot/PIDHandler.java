package org.usfirst.frc.team5827.robot;

import java.util.HashMap;

/**
 * A class to manage PID loops. It provides for management of multiple PID loops, and
 * switching back and forth between (and resetting) these loops.
 * 
 * @author Henry
 */

public class PIDHandler<PIDHandlerKey>
{
    // Store all loops in a hash table.
    private HashMap<PIDHandlerKey, PIDLoop> pidLoopContainer;
    private PIDHandlerKey currentKey;
    private boolean potentialForMultipleSelected;

    // A list of command names.
    public static enum Command { ENABLE, RESET, RESET_IF_ENABLED, DISABLE };

    /**
     * Make a new PIDHandler.
     */
    public PIDHandler()
    {
        pidLoopContainer = new HashMap<PIDHandlerKey, PIDLoop>();

        potentialForMultipleSelected = false;
        currentKey = null;
    }

    /**
     * Run a command for all PIDLoops.
     * 
     * @param command The name of the command to run.
     * @return True on success.
     */
    public boolean runCommandForAll(Command commandToRun)
    {
        boolean commandRun = false;

        // For every selected PIDLoop,
        for (PIDLoop currentLoop : pidLoopContainer.values())
        {
            // Run the selected command.
            switch(commandToRun)
            {
                case ENABLE:
                    //currentLoop.enable();
                    break;
                
                case RESET:
                    currentLoop.reset();
                    break;

                case RESET_IF_ENABLED:
                    //currentLoop.resetIfEnabled();
                    break;

                case DISABLE:
                    //currentLoop.disable();
                    break;
            }

            // Note that the command was run.
            commandRun = true;
        }

        return commandRun;
    }

    /**
     * Get the currently selected PID loop. This is protected, as
     * it should not be necessary to access the current PID loop
     * outside of the PIDHandler class.
     * 
     * @return The PIDLoop associated with currentKey.
     */
    protected PIDLoop getCurrentPIDLoop()
    {
        // If no loop is selected,
        if (currentKey == null)
        {
            // Note this.
            return null;
        }

        // Find the current loop and return it.
        PIDLoop currentLoop = pidLoopContainer.get(currentKey);

        return currentLoop;
    }

    /**
     * Enable the currently selected PID controller.
     * This method should return false on failure.
     * 
     * @return Returns true on success.
     */
    public boolean enable()
    {
        // If no PID loop is selected,
        if (currentKey == null)
        {
            // Do not enable.
            return false;
        }

        // Enable.
        PIDLoop current = getCurrentPIDLoop();
        //current.enable();

        // Return true on success.
        return true;
    }

    /**
     * Reset and disable the currently selected PID controller.
     * 
     * @return Returns false on failure.
     */
    public boolean reset()
    {
        // If no PID loop is selected,
        if (currentKey == null)
        {
            // Note failure.
            return false;
        }

        // Otherwise, enable.
        PIDLoop current = getCurrentPIDLoop();
        current.reset();

        // Return true to note success.
        return true;
    }

    /**
     * Set the setpoint of the currently selected PID controller.
     * @param desiredOutput is the output wanted of the input to the PIDLoop.
     * 
     * @return Returns false on failure.
     */
    public boolean setSetpoint(double desiredOutput)
    {
        // If no PID loop is selected,
        if (currentKey == null)
        {
            // Return false to note failure.
            return false;
        }

        // Otherwise, have the PIDLoop set the setpoint.
        PIDLoop current = getCurrentPIDLoop();
        current.setSetpoint(desiredOutput);

        // Note success.
        return true;
    }

    /**
     *  Get the output of the currently selected PID loop. If no loop is selected,
     * a warning message is logged.
     * 
     * @return The PID output of whichever PID loop is selected, or 0.0 if no loop is.
     */

    public double getOutput()
    {
        // If there is no selected PID loop.
        if (currentKey == null)
        {
            // Return 0.
            return 0.0;
        }

        // Otherwise, get the output of the current PID device.
        PIDLoop current = getCurrentPIDLoop();
        double result = current.getPIDOutput();

        // Return the PID output.
        return result;
    }

    /**
     * Select a PIDLoop from a given key. Do not enable the new loop.
     * DO NOT change deselectPIDLoop to call selectPIDLoop(null)
     * without first changing selectPIDLoop to prevent infinite recursion.
     * 
     * @param key The key to select.
     */
    public void selectPIDLoop(PIDHandlerKey key)
    {
        // Unselect the previous.
        deselectPIDLoop();

        // As long as there is a selected key (deselectPIDLoop should be used to
        //select no PID loop).
        if (key != null)
        {
            currentKey = key;
        }
        else
        {
            Logging.consoleLog("Warning: Key is null.");
        }
    }

    /**
     * Select a PIDLoop and ENABLE, using a given key.
     * Note: Calls selectPIDLoop(key).
     * 
     * @param key The key of the PIDLoop to select.
     * @param desiredOutput The desired output of the PIDLoop.
     */

    public void selectPIDLoop(PIDHandlerKey key, double desiredOutput)
    {
        selectPIDLoop(key); // Select the loop.

        setSetpoint(desiredOutput); // Set a setpoint for the PIDLoop.

        // Enable the chosen PIDLoop.
        enable();
    }

    /**
     * Deselect the current PID Loop.
     * This method checks to see if the current key is null before disabling. 
     */
    public void deselectPIDLoop()
    {
        if (currentKey != null)
        {
            // If there is only one selected PID loop,
            if(!potentialForMultipleSelected)
            {
                // Reset and disable the current PID loop.
                reset();
            }
            else
            {
                // Otherwise, reset them all.
                runCommandForAll(Command.RESET_IF_ENABLED);
            }

            // Deselect the current.
            currentKey = null;
        }
    }

    /**
     * Add a PID Loop to the list of PID Loops.
     * Fails if a loop with that name is already added
     * to prevent potential errors caused by the removal
     * of a loop before a reset/disable.
     * 
     * @return Successful adding of the new loop.
     */
    public boolean addPIDLoop(PIDHandlerKey key, PIDLoop loopToAdd)
    {
        if (pidLoopContainer.containsKey(key))
        {
            Logging.consoleLog("Warning: %s is already a key (from PIDHandler.java, addPIDLoop).", key.toString());

            // Note failure.
            return false;
        }

        // Add the loop to the list of PID loops.
        pidLoopContainer.put(key, loopToAdd);

        // Note success.
        return true;
    }

    /**
     * Note a potential group of selected PID loops. Call this before the use of multiple loops at a time.
     * * This will make deselectPIDLoop disable all enabled loops.
     */
    public void notePotentialForMultiplePIDLoopsSelected()
    {
        potentialForMultipleSelected = true;
    }

    /**
     * Get a stored PIDLoop.
     * 
     * @param key The key associated with the PIDLoop. If key is null, return the current PID Loop.
     * @return The associated PID Loop or null, if it does not exist.
     */
    public PIDLoop getPIDLoop(PIDHandlerKey key)
    {
        PIDLoop resultantLoop = null;

        if (key == null)
        {
            resultantLoop = getCurrentPIDLoop();
        } // Otherwise, if the key exists
        else if (pidLoopContainer.containsKey(key))
        {
            // Store the wanted loop in the resultantLoop.
            resultantLoop = pidLoopContainer.get(key);
        }

        return resultantLoop;
    }
}