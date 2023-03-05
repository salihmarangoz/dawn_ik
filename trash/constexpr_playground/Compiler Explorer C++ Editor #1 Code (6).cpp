#include <stdio.h>
#include <math.h>
#include <string>
#include <cassert>
#include <array>

const int num_joints = 39;
const int num_variables = 20;
const int num_links = 39;
const int num_collision_pairs = 136;

// Joint info
const std::string joint_names[39] = {"ASSUMED_FIXED_ROOT_JOINT","base_link_joint","base_link_to_cam0","base_link_to_cam1","base_link_to_cam2","base_link_to_cam3","base_link_to_platform","arm_holder_joint","arm_left_world_joint","arm_left_joint1","arm_left_joint2","arm_left_joint3","arm_left_joint4","arm_left_joint5","arm_left_joint6","arm_left_joint7","arm_left_joint_eef","arm_right_world_joint","arm_right_joint1","arm_right_joint2","arm_right_joint3","arm_right_joint4","arm_right_joint5","arm_right_joint6","arm_right_joint7","arm_right_joint_eef","head_world_joint","head_joint1","head_joint2","head_joint3","head_joint4","head_joint5","head_joint6","head_joint_eef","head_link_eef_camera_d405-link_fixed_joint","head_camera_d405-link_camera_d405-depth_fixed_joint","head_link_eef_camera_d435i-link_fixed_joint","head_camera_d435i-link_camera_d435i-depth_fixed_joint","base_link_to_tracker"};
const int joint_parent_link_idx[39] = {-1,0,1,1,1,1,1,6,7,8,9,10,11,12,13,14,15,7,17,18,19,20,21,22,23,24,7,26,27,28,29,30,31,32,33,34,33,36,1}; // -1 if no link available

// Link info
const std::string link_names[39] = {"world","base_link","camera0_link","camera1_link","camera2_link","camera3_link","platform_base","arm_holder_link","arm_left_link_base","arm_left_link1","arm_left_link2","arm_left_link3","arm_left_link4","arm_left_link5","arm_left_link6","arm_left_link7","arm_left_link_eef","arm_right_link_base","arm_right_link1","arm_right_link2","arm_right_link3","arm_right_link4","arm_right_link5","arm_right_link6","arm_right_link7","arm_right_link_eef","head_link_base","head_link1","head_link2","head_link3","head_link4","head_link5","head_link6","head_link_eef","head_camera_d405_link","head_camera_d405_color_optical_frame","head_camera_d435i_link","head_camera_d435i_color_optical_frame","tracker"};
const int link_parent_joint_idx[39] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38};

constexpr int endpoint_link_idx = 38;

///////////////////////////////////////////////////////////////////////////////////
// Computes the kinematic chain on compile time
///////////////////////////////////////////////////////////////////////////////////
/*
struct PartialChain
{
  int joint_idx[num_joints];
  int size;

  constexpr PartialChain(const int endpoint_link) : joint_idx(), size(0)
  {
    assert(endpoint_link < num_links && "endpoint_link is out of bounds");
    assert(endpoint_link >= 0 && "endpoint_link must be non-negative");
    int i=0;
    int j_idx = link_parent_joint_idx[endpoint_link];
    int chain_inv[num_joints] = {-1};
    bool check[num_joints] = {false};
    while (j_idx > 0)
    {
        assert(check[j_idx] == false && "detected loop in the kinematic tree");
        check[j_idx] = true;
        chain_inv[i] = j_idx;
        j_idx = link_parent_joint_idx[ joint_parent_link_idx[j_idx] ]; // find the parent joint
        i++;
    }
    size = i+1;
    assert(size <= num_joints && "partial chain can't be larger than the full chain");
    for (int i=0; i<size; i++)
    {
        joint_idx[i] = chain_inv[size-i-1];
    }
  }
};
*/
///////////////////////////////////////////////////////////////////////////////////

template <typename T, size_t N>
struct CeArray
{
    T arr[N];
    size_t size;

    // std::array initializer
    constexpr CeArray(const std::array<T,N>& in_arr) : arr(), size(N)
    {
        for (int i=0; i<N; i++) arr[i] = in_arr[i];
    }

    // empty initializer
    constexpr CeArray(): arr(), size(N){}

    // raw array initializer
    constexpr CeArray(const T(&in_arr)[N]) : arr(), size(N)
    {
        for (int i=0; i<N; i++) arr[i] = in_arr[i];
    }

    const T& operator[](size_t idx) const
    {
        return arr[idx];
    }

    /*
    T& operator[](size_t idx)
    {
        return arr[idx];
    }
    */
};

// countOf interface for CeArray
template<typename T, size_t N>
constexpr size_t countOf( CeArray<T,N> arr ) { return arr.size;}

// ResizeCeArray
template <typename newT, size_t newN>
struct ResizeCeArray : public CeArray<newT, newN>
{
    // raw array initializer
    template <typename oldT, size_t oldN>
    constexpr ResizeCeArray(const oldT(&in_arr)[oldN])
    {
        for (int i=0; i<std::min(oldN, newN); i++) this->arr[i] = in_arr[i];
    }

    // CeArray initializer
    template <typename oldT, size_t oldN>
    constexpr ResizeCeArray(const CeArray<oldT,oldN>& in_arr)
    {
        for (int i=0; i<std::min(oldN, newN); i++) this->arr[i] = in_arr.arr[i];
    }
};

// ReverseCeArray
template <typename T, size_t N>
struct ReverseCeArray : public CeArray<T, N>
{
    // raw array initializer
    constexpr ReverseCeArray(const T(&in_arr)[N])
    {
        for (int i=0; i<N; i++) this->arr[N-i-1] = in_arr[i];
    }

    // CeArray initializer
    constexpr ReverseCeArray(const CeArray<T,N>& in_arr)
    {
        for (int i=0; i<N; i++) this->arr[N-i-1] = in_arr.arr[i];
    }
};


template <typename T, size_t N>
struct InverseMapCeArray : public CeArray<T, N>
{
    // CeArray initializer
    constexpr InverseMapCeArray(const CeArray<T,N>& in_arr, const T invalid_value=-1, const bool allow_overwrite=false, const bool assume_invalid_on_error=false)
    {
        for (int i=0; i<N; i++) this->arr[i] = invalid_value;
        for (int i=0; i<N; i++)
        {
            if (in_arr.arr[i] == invalid_value) continue;
            if (assume_invalid_on_error && (in_arr[i] >= N || in_arr[i] < 0) ) continue;
            assert(in_arr.arr[i] < N && "Out of bounds. Maybe set assume_invalid_on_error=true ?"); 
            if (!allow_overwrite) assert((this->arr[in_arr.arr[i]] == invalid_value && "Overwriting is not allowed. Maybe set allow_overwrite=true ?"));
            this->arr[in_arr.arr[i]] = i;
        }
    }
};


template <typename T=int, size_t N=num_joints>
constexpr CeArray<T,N> PartialChain(const int endpoint_link)
{
    T arr[N] = {0};
    assert(endpoint_link < num_links && "endpoint_link is out of bounds");
    assert(endpoint_link >= 0 && "endpoint_link must be non-negative");
    int i=0;
    int j_idx = link_parent_joint_idx[endpoint_link];
    int chain_inv[N] = {-1};
    bool check[N] = {false};
    while (j_idx > 0)
    {
        assert(check[j_idx] == false && "detected loop in the kinematic tree");
        check[j_idx] = true;
        chain_inv[i] = j_idx;
        j_idx = link_parent_joint_idx[ joint_parent_link_idx[j_idx] ]; // find the parent joint
        i++;
    }
    int size = i+1;
    assert(size <= N && "partial chain can't be larger than the full chain");
    for (int i=0; i<size; i++)
    {
        arr[i] = chain_inv[size-i-1];
    }
    return CeArray<T,N>(arr);
}



// util: find number of elements in an static array
#include <stddef.h>
template<class Type, ptrdiff_t n>
ptrdiff_t countOf( Type (&)[n] ) { return n; }

template<typename T, size_t N>
size_t countOf( const std::array<T,N> &arr ) { return N;}

// Try to use countOf instead, use this if other option doesn't compile
#ifndef COUNTOF
#define COUNTOF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#else
#pragma message "COUNTOF is already defined. Just make sure that other definition does the same thing"
#endif


int main()
{
    /*
    constexpr PartialChain c(endpoint_link_idx);
    constexpr int partial_num_joints = c.size;
    constexpr int partial_chain[c.size] = {c.joint_idx};

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
    */

    constexpr int test_arr[] = {0,4,3,1,2};

    { // TEST: raw -> CeArray
        constexpr auto arr = CeArray(test_arr);
        printf("size: %d\n", arr.size);
        for (int i=0; i<countOf(arr); i++){printf("%d ", arr[i]);}
        printf("\n");
    }

    { // TEST: CeArray -> ResizeCeArray
        constexpr auto arr = ResizeCeArray<float,10>(CeArray(test_arr));
        printf("size: %d\n", arr.size);
        for (int i=0; i<countOf(arr); i++){printf("%.1f ", arr[i]);}
        printf("\n");
    }

    { // TEST: raw -> ResizeCeArray
        constexpr auto arr = ResizeCeArray<float,3>(test_arr);
        printf("size: %d\n", arr.size);
        for (int i=0; i<countOf(arr); i++){printf("%.1f ", arr[i]);}
        printf("\n");
    }

    { // TEST CeArray -> InverseMapCeArray -> 0,4,3,1,2 -> 0,3,4,2,1
        constexpr auto arr = InverseMapCeArray(CeArray(test_arr));
        printf("size: %d\n", arr.size);
        for (int i=0; i<countOf(arr); i++){printf("%d ", arr[i]);}
        printf("\n");
    }

    { // TEST CeArray -> InverseMapCeArray
        constexpr auto arr = InverseMapCeArray(CeArray(test_arr));
        printf("size: %d\n", arr.size);
        for (int i=0; i<countOf(arr); i++){printf("%d ", arr[i]);}
        printf("\n");
    }

    {
        constexpr auto c = PartialChain(endpoint_link_idx);
        //constexpr int partial_num_joints = c.size;
        //constexpr int partial_chain[c.size] = {c.joint_idx};
        for (int i=0; i<num_links; i++)
        {
            printf("idx: %d link: %s\n", i, link_names[i].c_str());
        }

        printf("selected link: %s\n", link_names[endpoint_link_idx].c_str());

        for (int i=0; i<c.size; i++)
        {
            printf("joint_names: %s\n", joint_names[c.arr[i]].c_str());
            printf("joint_idx: %d\n", c.arr[i]);
        }
    }


    return 0;
}