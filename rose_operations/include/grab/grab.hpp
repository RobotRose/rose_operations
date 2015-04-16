/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/14
* 		- File created.
*
* Description:
*	Grab class. Grabs item stored in the database. Does basic visual servoing.
* 
***********************************************************************************/
#ifndef GRAB_HPP
#define GRAB_HPP

#include <unistd.h> //sleep

#include "manipulation_base_class/manipulation_base_class.hpp"
#include "tf_helper/tf_helper.hpp"

#define DO_VISUAL_SERVOING true
#define PRE_GRAB_X        0.07  // in meters
#define MIN_PRE_GRAB_X    0.13  // in meters
#define MIN_GRIPPER_TABLE_DIST_DOWN 0.065 // in meters
#define MIN_GRIPPER_TABLE_DIST_FORW 0.0 // in meters
#define POST_GRAB_Z       0.05  // in meters 
#define TABLE_CORRECTION  0.02
#define TABLE_HEIGHT      2.00  // (twice extent in simulator robai) in meters
#define BASE_LINK_LENGTH  0.64

/**
 * Grab class. Retrieves some functionality from the OperationBaseClass
 */
class Grab : public ManipulationBaseClass
{
  public:
    typedef actionlib::SimpleActionClient<rose_gaze_controller::LookAtAction> GazeClient; //! @todo MdL: Make SMC.

    /**
     * Constructor
     */
	Grab( std::string name, ros::NodeHandle n );

    /**
     * Destructor
     */
	~Grab();

  private:
    /**
     * Callback when a cancel was retrieved
     * @param smc A pointer to the SMC receiving the cancel
     */
    void CB_serverCancel( SMC* smc );

    /**
     * Executes this function when a goal for an Item has been recieved
     * @param item_id    For which item_id the goal is received
     * @param parameters Possible additional parameters
     */
    void executeItemAction( const std::string item_id, const vector<std::string> parameters = vector<std::string>() );

    /**
     * This function continues with the item action.
     * It given to the getParameter fuction as the continuation function after retrieving the parameters
     * @param item_id    For which item_id the goal is received (usually the same of the executeItemAction function 
     *                   where getParameter was executed)
     * @param parameters Possible additional parameters
     */
    void continueItemAction( const std::string item_id, const vector<std::string> parameters = vector<std::string>() );

    /**
     * Function that executes all prerequisites before doing the actual grabbing action.
     * For instance: It creates a transform for the Item with item_id
     * @param item_id    The item that has to be grabbed
     * @param parameters Additional parameters
     */
    void executeGrabItem( const std::string item_id, const vector<std::string> parameters );

    /**
     * Gets the pose, relative (x,y,z) in base_link to the grabbable object
     * @param  item The grabbable object
     * @param  pose The pose to retrieve
     * @param  x    Add x (in base_link frame)
     * @param  y    Add y (in base_link frame)
     * @param  z    Add y (in base_link frame)
     * @return      Whether of not this function was successful
     */
    bool getPoseRelativeToGrab( Item item, PoseStamped& pose, const double x, const double y, const double z );

    /**
     * Gets the pre grab pose specifically for item
     * @param  item The item to pre grab
     * @param  pose The pose to retrieve
     * @return      Whether of not this function was successful
     */
    bool getPreGrabPose( Item item, PoseStamped& pose );

    /**
     * Gets the post grab pose specifically for item
     * @param  item The item to post grab
     * @param  pose The pose to retrieve
     * @return      Whether of not this function was successful
     */
    bool getPostGrabPose( Item item, PoseStamped& pose );

    bool getGrabPose( Item item, PoseStamped& pose );

    /**
     * Gets the table pose specifically for item
     * @param  item The table pose for item
     * @param  pose The pose to retrieve
     * @return      Whether of not this function was successful
     */

    bool preGrab( const std::string arm_name, Item item );
    /**
     * Execute grab action
     * @param  arm  The arm to do the grab action
     * @param  item The item to grab
     * @return      If the action was successful
     */
    bool grab( const std::string arm_name, Item item );

    /**
     * Execute post grab action
     * @param  arm  The arm to do the post grab action
     * @param  item The item to post grab
     * @return      If the action was successful
     */
    bool postGrab( const std::string arm_name, Item item );

    /**
     * Visual servoing towards item
     * @param  arm  Which arm
     * @param  item Which item
     * @return      If the action was successful
     */
    bool moveTowardsItem( const std::string arm_name, Item item );

    /**
     * Removed all added stuff for the grab to work.
     * Removes created transform, bounding box of item
     * @param item The item information
     */
    void cleanup( Item item );

    /**
     * Sends a result message and cleans up all created stuff
     * @param result     If the grab was successful or not
     * @param item      The item on which the grab was executed
     * @param message   Specific error/succes message
     */
    void sendResultAndCleanup( bool result, Item item, std::string message );

    void grabSuccess( Item item );
    void grabFailed( Item item, std::string message);
};

#endif //GRAB_HPP
