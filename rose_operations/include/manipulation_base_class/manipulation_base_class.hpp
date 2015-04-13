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
    // typedef ServerMultipleClient<rose_arm_controller_msgs::move_to_tfAction> ArmVisualServoing;

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
     * Sends a visual servoing goal to the visual servoing node. Waits and retrieves the result. It also sends
     * an aborted message as a server when the goal has failed. 
     * @param  goal The goal for the visual servoing node
     * @return      If the action was successful
     */
    // bool visualServoingAction ( const rose_arm_controller_msgs::move_to_tfGoal goal );

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
