#include <salih_marangoz_thesis/robot_parser.h>

namespace salih_marangoz_thesis
{

RobotParser::RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) : nh(nh), priv_nh(priv_nh)
{

}

std::string RobotParser::createHeader()
{
  return "";
}

std::string RobotParser::createFooter()
{
  return "";
}

template <typename T>
std::string RobotParser::vector2Str(const std::string& variable, const std::vector<T>& arr)
{
  std::string out = "";
  out += variable;
  out += "[] = {";

  for (int i=0; i<arr.size(); i++)
  {
    out += std::to_string(arr[i]);

    if (i!=arr.size()-1)
    {
      out += ",";
    }
    else
    {
      out +="};";
    }
  }

  return out;
}

/*
  const int x[3][4] = {{0,1,2,3}, 
                       {4,5,6,7}, 
                       {8,9,10,11}};
*/
std::string RobotParser::eigenTranslation2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations)
{
  auto translation = transformations.translation();
  //auto rotation = Eigen::Quaterniond(transformation.rotation());

  std::string out = "";
  out += variable;
  out += "[";
  out += std::to_string(transformations.size());
  out += "]";
  out += "[4] = {";
  int empty_spaces = out.size(); // std::string(empty_spaces, ' ')

  for (int i=0; i<transformations.size(); i++)
  {
    
  }


  return out;
}


void parseCurrentRobot()
{
  moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();

  // Follow the kinematic tree
  const moveit::core::JointModel* current_joint = robot_model->getRootJoint();
  ROS_INFO("Joint: %s", current_joint->getName().c_str());

  float pos[3] = {0,0,0};
  float rot[4] = {1,0,0,0};

  while (true)
  {
    // One child link is possible
    const moveit::core::LinkModel* current_link = current_joint->getChildLinkModel();
    ROS_INFO("Link: %s", current_link->getName().c_str());

    // Get link transform
    const Eigen::Isometry3d current_link_transform = current_link->getJointOriginTransform();
    if (!current_link_transform.translation().isZero())
    {
      ROS_INFO_STREAM("translation: " << current_link_transform.translation());

      float link_translation[3];
      link_translation[0] = current_link_transform.translation().x();
      link_translation[1] = current_link_transform.translation().y();
      link_translation[2] = current_link_transform.translation().z();

      float out[3];
      utils::computeLinkTranslation(pos, rot, link_translation, out);
      pos[0] = out[0];
      pos[1] = out[1];
      pos[2] = out[2];
    }

    if (current_link_transform.rotation().isIdentity())
    {
      ROS_INFO_STREAM("rotation: " << current_link_transform.rotation() << " without joint rotation");
    }
    else
    {
      ROS_INFO_STREAM("rotation: " << current_link_transform.rotation() << " with joint rotation");
    }

    const double* pos_ = robot_state.getJointPositions(current_joint->getName());
    float pos = *pos_;

    Eigen::Quaterniond link_rot(current_link_transform.rotation());
    float link_rot_r[4];
    link_rot_r[0] = link_rot.w();
    link_rot_r[1] = link_rot.x();
    link_rot_r[2] = link_rot.y();
    link_rot_r[3] = link_rot.z();

    float out[4];
    utils::computeLinkRotation(rot, link_rot_r, pos, out);
    rot[0] = out[0];
    rot[1] = out[1];
    rot[2] = out[2];
    rot[3] = out[3];

    // Multiple joints are possible. But here we will pick the first one.
    const std::vector<const moveit::core::JointModel*> next_joints = current_link->getChildJointModels();
    if (next_joints.size() <= 0) break; // end of the kinematic tree
    current_joint = next_joints[0];
    ROS_INFO("Joint: %s", current_joint->getName().c_str());
  }

  geometry_msgs::PoseStamped p;
  p.pose.position.x = pos[0];
  p.pose.position.y = pos[1];
  p.pose.position.z = pos[2];
  p.pose.orientation.w = rot[0];
  p.pose.orientation.x = rot[1];
  p.pose.orientation.y = rot[2];
  p.pose.orientation.z = rot[3];
  p.header.frame_id = "world";
  p.header.stamp = ros::Time::now();
  return p;
}



} // namespace salih_marangoz_thesis