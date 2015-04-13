/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/14
* 		- File created.
*
* Description:
*	ManipulationBaseClass class. ManipulationBaseClasss item stored in the database. Does basic visual servoing.
* 
***********************************************************************************/
#ifndef MANIPULATION_BASE_CLASS_HPP
#define MANIPULATION_BASE_CLASS_HPP

#include <unistd.h> //sleep

#include "operation_base_class/operation_base_class.hpp"
#include "tf_helper/tf_helper.hpp"

#include "rose_gaze_controller/LookAtAction.h" //! @todo MdL: Coding guidelines (lower case filenames).
#include "rose_gaze_controller/LookAtGoal.h"   //! @todo MdL: Coding guidelines (lower case filenames).

using geometry_msgs::PoseStamped;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using std::vector;
using std::map;

#define MIN_GRIPPER_TABLE_DIST 0.08 // in meters
#define TABLE_HEIGHT      2.00  // (twice extent in simulator robai) in meters
#define BASE_LINK_LENGTH  0.64

/**
 * ManipulationBaseClass class. Retrieves some functionality from the OperationBaseClass
 */
class ManipulationBaseClass : public OperationBaseClass
{
  public:
    typedef actionlib::SimpleActionClient<rose_gaze_controller::LookAtAction> GazeClient; //! @todo MdL: Make SMC.
    typedef ServerMultipleClient<rose_arm_controller_msgs::move_to_tfAction> ArmVisualServoing;

    /**
     * Constructor
     */
	ManipulationBaseClass( std::string name, ros::NodeHandle n );

    /**
     * Destructor
     */
    ~ManipulationBaseClass();

    /**
     * Broadcasts all stored transforms. These are items that are stored for grabbing. This is needed 
     * for the visual servoing functionality and this function is executed by the grab node (grab_node.cpp)
     */
    void broadcastTransforms();

  protected:
    /**
     * Picks the best arm for a list of poses (which arm can reach the most?)
     * @param  poses The list of poses
     * @return       The arm chosen
     */
    ArmController::Arms pickArm( const vector<PoseStamped> poses );

    /**
     * Check is all poses in a list are reachable by a certain arm.
     * @param  ArmController::Arms Arm
     * @param  poses               List of poses
     * @return                     Whether or not all poses are reachable by arm arm
     */
    bool reachable ( const ArmController::Arms, const vector<PoseStamped> poses );

    /** 
     * Returns the number of poses in a list that are reachable by a certain arm
     * @param  arm   Which arm
     * @param  poses Which poses
     * @return       The number of reachable poses in the list of poses
     */
    int getNrOfReachablePoints( const ArmController::Arms arm, const vector<PoseStamped> poses );
    
    /**
     * Creates a table pose just below the bounding box of an item
     * @param bb The bounding box
     */
    void createTablePose( const BoundingBox& bb );

    /**
     * Send a arm controller goal to the arm server. Waits and retrieves the result. It also sends an aborted message
     * as a server when the goal has failed
     * @param  arm  The arm to send the goal to
     * @param  goal The goal message
     * @return      If the action was successful
     */
    bool armAction ( const ArmController::Arms arm, const arm_controller::manipulateGoal goal );

    /**
     * Sends a visual servoing goal to the visual servoing node. Waits and retrieves the result. It also sends
     * an aborted message as a server when the goal has failed. 
     * @param  goal The goal for the visual servoing node
     * @return      If the action was successful
     */
    bool visualServoingAction ( const rose_arm_controller_msgs::move_to_tfGoal goal );

    /**
     * Creates transform with name and pose_stamped
     * @param name         Name of the transform
     * @param pose_stamped Pose where the the transform should be
     */
    void createTransform( const std::string name, const PoseStamped pose_stamped );
    /**
     * Updates transform to a new pose
     * @param name         Name of the transform to update
     * @param pose_stamped New pose
     */
    void updateTransform( const std::string name, const PoseStamped pose_stamped );

    /**
     * Removes transform from list
     * @param name Name of the transform
     */
    void removeTransform( const std::string name );

    /**
     * Gets the table pose specifically for item
     * @param  item The table pose for item
     * @param  pose The pose to retrieve
     * @return      Whether of not this function was successful
     */
    bool getTablePose( Item item, PoseStamped& pose );

    /**
     * Execute pre grab action
     * @param  arm  The arm to do the pre grab action
     * @param  item The item to pre grab
     * @return      If the action was successful
     */
    bool preManipulationBaseClass( const ArmController::Arms arm, Item item );

    /**
     * Open the gripper of arm
     * @param  arm Which arm
     * @return     If the action was successful
     */
    bool openGripper( const ArmController::Arms arm );

    /**
     * Close the gripper of arm
     * @param  arm Which arm
     * @return     If the action was successful
     */
    bool closeGripper( const ArmController::Arms arm );

    /**
     * Moves the gripper of arm up with POST_MANIPULATION_BASE_CLASS_Z
     * @param  arm  Which arm
     * @return      If the action was successful
     */
    bool moveGripperUp( const ArmController::Arms arm, const double z, bool constrained=false);

    /**
     * Removes the bounding box of an item from the database
     * @param item For which item the bounding box should be removed
     */
    void removeBoundingBoxOfItemFromDatabase( Item item );

    /**
     * Looks at a point
     * todo MdL: Move to generic class
     * @param  frame_id      The origin of which frame to look to
     * @param  keep_tracking Keep tracking?
     * @return               If the command was successfully issued
     */
    const bool lookAt( const std::string frame_id, const bool keep_tracking=false);

    GazeClient*         gaze_client_;               //!< Client to look at a specific point
    ArmVisualServoing*  arm_visual_servoing_;       //!< The visual servoing server

    ros::ServiceClient  toggle_visual_correction_service_; //!< Service to call to toggle visual correction (not needed/used anymore)
    ros::ServiceClient  reset_visual_correction_service_;  //!< Service to call to reset  visual correction (not needed/used anymore) 

    map<std::string, TFHelper*>   transforms_;      //!< All transforms stored by this class

    boost::mutex        transforms_mutex_;          //!< Transforms are TFHelper pointers
};

#endif //MANIPULATION_BASE_CLASS_HPP
