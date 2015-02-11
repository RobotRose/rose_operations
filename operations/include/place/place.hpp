/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/14
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef PLACE_HPP
#define PLACE_HPP

#include "arm_controller_helper.hpp"
#include "manipulation_base_class/manipulation_base_class.hpp"
#include "tf_helper/tf_helper.hpp"
#include <iostream>

#define PRE_PLACE_Z_OFFSET 0.10//[m]
#define POST_PLACE_X_OFFSET -0.10 //[m]
#define POST_PLACE_Z_OFFSET 0.05 //[m]

class ManipulationBaseClass;

class Place : public ManipulationBaseClass
{
  public:
	Place( std::string name, ros::NodeHandle n );
    ~Place();

  private:
    const std::string operator_defined_waypoint;

    void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
    void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );

    void executeItemAction( const std::string item_id, const std::vector<std::string> parameters = std::vector<std::string>() );
    void executeMultiItemAction( const std::vector<std::string> item_ids, const std::vector<std::string> parameters = std::vector<std::string>() );
    void continueItemAndWaypoint( const std::string item_id, const std::string waypoint_id, const std::vector<std::string> parameters = std::vector<std::string>() );

    void continuePlaceAction( const ArmController::Arms arm, const std::string item_id, const Waypoint place_location, const std::vector<std::string> parameters = std::vector<std::string>() );

    /**
     * Function that executes all prerequisites before doing the actual placing action.
     * @param item_id    The item that has to be places
     * @param place_location the position to place the item
     * @param parameters Additional parameters
     */
    void prePlaceAction( const std::string item_id, const Waypoint place_location, const vector<std::string> parameters );
    void placeAction( const std::string item_id, const Waypoint place_location );

    bool prePlace( const ArmController::Arms arm, const Waypoint place_location, Item item );
    bool place( const ArmController::Arms arm, const Waypoint place_location, Item item );
    bool postPlace( const ArmController::Arms arm, const Waypoint place_location, Item item );
    bool moveTowardPlacement(const ArmController::Arms arm, const Waypoint place_location, Item item);

	void removeBoundingBoxOfItemFromDatabase( Item item );
    void removeLocationFromDatabase( Waypoint waypoint );

    /**
     * Gets the pre place pose specifically for item. The item will be handing in the air, just above the intended place position
     * This means moving the grippertip, which is has the item by its center (approximately), to:
     *
     *     some static offset
     *     +
     *     1/2 object height
     *     +
     *     Z of the place_position (projected to base_link)
     *
     * @param  item The item to place
     * @param  place_location where to put the item
     * @param  pose The pose to return
     * @return      Whether of not this function was successful
     */
    bool getPrePlacePose( Item item, const Waypoint place_location, PoseStamped& pose );

    /**
     * @param  place the pose where to put the table's center, for collision avoidance in the arms
     * @return       Whether of not this function was successful
      */
    bool insertTableAtPlace(const ArmController::Arms arm, PoseStamped place);

    /**
     * @param  place the pose where to put the item's center, for collision avoidance in the arms
     * @return       Whether of not this function was successful
      */
    bool insertItemAtPlace(const ArmController::Arms arm, PoseStamped item);

    /**
     * Get the pose for the gripper at the moment of placing (i.e. the item and the table just make contact)
     * @param  item The item to place
     * @param  place_location where to put the item
     * @param  pose The pose to return
     * @return      Whether of not this function was successful
     */
    bool getPlacePose( Item item, const Waypoint place_location, PoseStamped& pose );

    void cleanup( Item item, Waypoint waypoint );

    /**
     * Sends a result message and cleans up all created stuff
     * @param result    If the grab was successful or not
     * @param item      The item on which the grab was executed
     * @param message   A specific message to display to the user.
     */
    void sendResultAndCleanup( bool result, Item item, Waypoint waypoint, std::string message);

    TFHelper* goal_tf_;
    std::map<ArmController::Manipulators, geometry_msgs::PoseStamped> obstacles;
};

#endif //PLACE_HPP
