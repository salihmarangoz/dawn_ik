#include <stdio.h>
#include <math.h>
#include <string>

const int num_joints = 39;
const int num_variables = 20;
const int num_links = 39;
const int num_collision_pairs = 136;

// Mapping vectors
const int joint_idx_to_variable_idx[39] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,0,1,2,3,4,5,6,-1,-1,7,8,9,10,11,12,13,-1,-1,14,15,16,17,18,19,-1,-1,-1,-1,-1,-1}; // -1 if no variable available. Can be used as joint_has_variable vector
const int variable_idx_to_joint_idx[20] = {9,10,11,12,13,14,15,18,19,20,21,22,23,24,27,28,29,30,31,32};

// Joint info
const std::string joint_names[39] = {"ASSUMED_FIXED_ROOT_JOINT","base_link_joint","base_link_to_cam0","base_link_to_cam1","base_link_to_cam2","base_link_to_cam3","base_link_to_platform","arm_holder_joint","arm_left_world_joint","arm_left_joint1","arm_left_joint2","arm_left_joint3","arm_left_joint4","arm_left_joint5","arm_left_joint6","arm_left_joint7","arm_left_joint_eef","arm_right_world_joint","arm_right_joint1","arm_right_joint2","arm_right_joint3","arm_right_joint4","arm_right_joint5","arm_right_joint6","arm_right_joint7","arm_right_joint_eef","head_world_joint","head_joint1","head_joint2","head_joint3","head_joint4","head_joint5","head_joint6","head_joint_eef","head_link_eef_camera_d405-link_fixed_joint","head_camera_d405-link_camera_d405-depth_fixed_joint","head_link_eef_camera_d435i-link_fixed_joint","head_camera_d435i-link_camera_d435i-depth_fixed_joint","base_link_to_tracker"};
const int joint_child_link_idx[39] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38};
const int joint_parent_link_idx[39] = {-1,0,1,1,1,1,1,6,7,8,9,10,11,12,13,14,15,7,17,18,19,20,21,22,23,24,7,26,27,28,29,30,31,32,33,34,33,36,1}; // -1 if no link available

// Link info
const std::string link_names[39] = {"world","base_link","camera0_link","camera1_link","camera2_link","camera3_link","platform_base","arm_holder_link","arm_left_link_base","arm_left_link1","arm_left_link2","arm_left_link3","arm_left_link4","arm_left_link5","arm_left_link6","arm_left_link7","arm_left_link_eef","arm_right_link_base","arm_right_link1","arm_right_link2","arm_right_link3","arm_right_link4","arm_right_link5","arm_right_link6","arm_right_link7","arm_right_link_eef","head_link_base","head_link1","head_link2","head_link3","head_link4","head_link5","head_link6","head_link_eef","head_camera_d405_link","head_camera_d405_color_optical_frame","head_camera_d435i_link","head_camera_d435i_color_optical_frame","tracker"};
const int link_parent_joint_idx[39] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38};

const int endpoint_link_idx = 33;

///////////////////////////////////////////////////////////////////////////////////
// Computes the kinematic chain on compile time
///////////////////////////////////////////////////////////////////////////////////
struct PartialChain{
  int joint_idx[num_joints];
  int size;

  constexpr PartialChain(const int endpoint_link) : joint_idx(), size(0){
    int i=0;
    int j_idx = link_parent_joint_idx[endpoint_link];
    int chain_inv[num_joints] = {-1};
    while (j_idx > 0)
    {
        chain_inv[i] = j_idx;
        j_idx = link_parent_joint_idx[ joint_parent_link_idx[j_idx] ];
        i++;
    }
    size = i+1;
    for (int i=0; i<size; i++)
    {
        joint_idx[i] = chain_inv[size-i-1];
    }
  }
};
///////////////////////////////////////////////////////////////////////////////////


int main()
{
    constexpr PartialChain c(endpoint_link_idx);

    for (int i=0; i<num_links; i++)
    {
        printf("idx: %d link: %s\n", i, link_names[i].c_str());
    }

    printf("selected link: %s\n", link_names[endpoint_link_idx].c_str());

    for (int i=0; i<c.size; i++)
    {
        printf("joint_names: %s\n", joint_names[c.joint_idx[i]].c_str());
        //printf("joint_idx: %d\n", c.joint_idx[i]);
    }

    return 0;
}