#ifndef ROS_NODE_MPC_CONTROLLER_H
#define ROS_NODE_MPC_CONTROLLER_H

/**
 * Default node name
 */
#define ROS_NODE_MPC_CONTROLLER_NODE_NAME "ur5_nmpc_controller_node"

/**
 * Default result (publisher) topic name
 */
#define ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC "result"

/**
 * Default parameters (subscriber) topic name
 */
#define ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC "parameters"

/**
 * Default execution rate (in Hz)
 */
#define ROS_NODE_MPC_CONTROLLER_RATE 6

/**
 * Default result topic queue size
 */
#define ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC_QUEUE_SIZE 100

/**
 * Default parameters topic queue size
 */
#define ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC_QUEUE_SIZE 100

/**
 * Default initial penalty
 */
#define ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY 1.0


#endif /* Header Sentinel: MPC_DUMMY_H */