 #include <kdl_parser/kdl_parser.hpp>
 
  KDL::Tree my_tree;
   ros::NodeHandle node;
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, string());
   if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
