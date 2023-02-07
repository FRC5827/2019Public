package org.usfirst.frc.team5827.robot.dashboard_manager;

/**
 * A class to manage data sent to the smart dashboard and received from it.
 * 
 * Goal:
 *  - Classes can implement DashboardOutputter, notify the SmartDashboardManager
 *    of their creation, and have a method called periodically that retrieves information
 *    from and posts information to the smart dashboard.
 * 
 * Notes:
 *  - I am not sure I like how this will be set up. Please feel free to comment on this method
 *    of organization! A single method with calls to put and get things from the smart dashboard
 *    might be simpler and easier to maintain. 
 *  - JavaDoc comments are not used inside this class, in an attempt to improve readability.
 *  - Why even have this class? The Shuffleboard supports tabs. This doesn't do anything with
 *    them yet, but, eventually, clients should request a tab for output.
 * (Oops! SmarterDashboard was deleted. This will replace it.
 * This can be re-named to SmarterDashboard, and should be the same).
 * 
 * @author Henry
 */

import java.util.ArrayList;

/*
Public Methods:
    public static SmartDashboardManager getInstance(); // Get an instance of this SmartDashboardManager (allows the existence of just one manager).
    public void addObserver(DashboardOutputter observer); // Add an object that can put information on the smart dashboard. TODO: Consider changing this method name. This might not be the observer pattern.
    public void update(); // To be called periodically to update information on the smart dashboard.
*/

public class SmartDashboardManager
{
    private static SmartDashboardManager instance = null; // The ONE instance of SmartDashboardManager

    private ArrayList<DashboardOutputter> clients;

    // This constructor is private. Only
    //one instance of SmartDashboardManager
    //should exist. Call getInstance instead.
    private SmartDashboardManager()
    {
        // Initialize variables.
        clients = new ArrayList<DashboardOutputter>();

        // Note: instance is initialized in getInstance.
    }

    // Get a SmartDashboardManager. If none exists,
    //create a new one. Only one SmartDashboardManager should
    //exist.
    public static SmartDashboardManager getInstance()
    {
        SmartDashboardManager result;

        if (instance != null)
        {
            result = instance;
        }
        else
        {
            result = new SmartDashboardManager();

            // Note the creation of the manager.
            instance = result;
        }

        return result;
    }

    // Add a client to the SmartDashboardManager,
    //that implements the DashboardOutputter interface.
    public void addObserver(DashboardOutputter observer)
    {
        clients.add(observer);

        observer.initializeDashboard();
    }

    // Update all clients with a refrence to the dashboard to be used.
    public void update()
    {
        for(DashboardOutputter outputter : clients)
        {
            outputter.updateDashboard();
        }
    }
}