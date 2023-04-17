
/*
#include <Eigen/Geometry>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/internal/tools.h>
#include <hpp/fcl/broadphase/broadphase.h>
#include <random>

using hpp::fcl::Box;
using hpp::fcl::Sphere;
using hpp::fcl::collide;
using hpp::fcl::CollisionRequest;
using hpp::fcl::CollisionResult;
using hpp::fcl::ComputeCollision;
using hpp::fcl::FCL_REAL;
using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;
using hpp::fcl::CollisionGeometryPtr_t;
using hpp::fcl::CollisionObject;
using hpp::fcl::DynamicAABBTreeCollisionManager;
using hpp::fcl::NaiveCollisionManager;

using namespace std::chrono;

// TODO: use FCL_REAL

FCL_REAL rand_interval(FCL_REAL rmin, FCL_REAL rmax) {
  FCL_REAL t = rand() / ((FCL_REAL)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

void eulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Eigen::Matrix3d& R) {
  FCL_REAL c1 = cos(a);
  FCL_REAL c2 = cos(b);
  FCL_REAL c3 = cos(c);
  FCL_REAL s1 = sin(a);
  FCL_REAL s2 = sin(b);
  FCL_REAL s3 = sin(c);

  R << c1 * c2, -c2 * s1, s2, c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3,
      -c2 * s3, s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

void generateRandomTransform(FCL_REAL extents[6], Transform3f& transform) {
  FCL_REAL x = rand_interval(extents[0], extents[3]);
  FCL_REAL y = rand_interval(extents[1], extents[4]);
  FCL_REAL z = rand_interval(extents[2], extents[5]);

  const FCL_REAL pi = 3.1415926;
  FCL_REAL a = rand_interval(0, 2 * pi);
  FCL_REAL b = rand_interval(0, 2 * pi);
  FCL_REAL c = rand_interval(0, 2 * pi);

  Eigen::Matrix3d R;
  eulerToMatrix(a, b, c, R);
  Vec3f T(x, y, z);
  transform.setTransform(R, T);
}


struct CollisionCallBackCollect : hpp::fcl::CollisionCallBackBase {
  typedef std::pair<CollisionObject*, CollisionObject*> CollisionPair;
  CollisionCallBackCollect();
  bool collide(CollisionObject* o1, CollisionObject* o2);
  size_t numCollisionPairs() const;
  const std::vector<CollisionPair>& getCollisionPairs() const;
  void init();
  bool exist(const CollisionPair& pair) const;
  virtual ~CollisionCallBackCollect(){};

 protected:
  std::vector<CollisionPair> collision_pairs;
  size_t max_size;
};

CollisionCallBackCollect::CollisionCallBackCollect(){}

bool CollisionCallBackCollect::collide(CollisionObject* o1, CollisionObject* o2) {
  collision_pairs.push_back(std::make_pair(o1, o2));
  return false;
}

size_t CollisionCallBackCollect::numCollisionPairs() const {
  return collision_pairs.size();
}

const std::vector<CollisionCallBackCollect::CollisionPair>&
CollisionCallBackCollect::getCollisionPairs() const {
  return collision_pairs;
}

void CollisionCallBackCollect::init() { collision_pairs.clear(); }

bool CollisionCallBackCollect::exist(const CollisionPair& pair) const {
  return std::find(collision_pairs.begin(), collision_pairs.end(), pair) != collision_pairs.end();
}

int main()
{
  srand(time(NULL));
  NaiveCollisionManager  manager;
  //DynamicAABBTreeCollisionManager manager;
  //hpp::fcl::SSaPCollisionManager manager;
  std::vector<CollisionObject*> objects;
  CollisionCallBackCollect collision_callback;

  // NOTE: Inflated shape transform difference (.second) is only implemented for Cone. Let's not support this shape.
  // Sphere s1(0.1);
  // auto inflated_shape1 = s1.inflated(0.1);
  // Sphere s2(0.1);
  // auto inflated_shape2 = s2.inflated(0.1);
  int N=10000;
  for (int i=0; i<N; i++)
  {
    //Box s1(0.05, 0.07, 0.1);
    hpp::fcl::Capsule s1(0.05, 0.05);
    auto inflated_shape1 = s1.inflated(0.01);
    CollisionGeometryPtr_t collision_shape = std::make_shared<hpp::fcl::Capsule>(inflated_shape1.first);
    CollisionObject* collision_object = new CollisionObject(collision_shape);

    Transform3f transform;
    FCL_REAL extents[6] = {0,0,0,5,5,5};
    generateRandomTransform(extents, transform);
    //const Eigen::Vector3d position0(0.1, 0.2, 0.3);
    collision_object->setTransform(transform);
    collision_object->computeAABB();

    objects.push_back(collision_object);
  }

  manager.registerObjects(objects);
  manager.update();

  for (int i=0; i<5; i++)
  {
    auto start = high_resolution_clock::now();

    for (int i=0; i<N; i++)
    {
      CollisionObject* collision_object = objects[i];
      collision_object->setTranslation(collision_object->getTranslation() + hpp::fcl::Vec3f(rand_interval(0, 0.05), rand_interval(0, 0.05), rand_interval(0, 0.05)));
      collision_object->computeAABB();
    }
    manager.update();

    manager.collide(&collision_callback);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    printf("timing: %d\n", duration);
    printf("total collisions: %d\n", collision_callback.numCollisionPairs());  
  }

}
*/



// 100000 capsule-capsule computations -> 0.057 seconds

// int main()
// {
//     hpp::fcl::Sphere shape1(0.1);
//     //hpp::fcl::Capsule shape2(0.1, 0.2);
//     hpp::fcl::Box shape2(0.1, 0.2, 0.3);

//     Eigen::Vector3d v1(0,0,0);
//     Eigen::Vector3d v2(1,0,0);
//     Transform3f T1(v1);
//     Transform3f T2(v2);

//     CollisionRequest req;
//     req.distance_upper_bound = 0.3;
//     //req.collision_distance_threshold = 0.1;
//     //req.security_margin = 0.1;

//     CollisionResult res;
//     collide(&shape1, T1, &shape2, T2, req, res);
//     printf("%f\n", res.distance_lower_bound);
// }

// int main()
// {
//   std::random_device rd;
//   std::mt19937 e2(rd());
//   std::uniform_real_distribution<> dist(-1, 1);

//   int total_collisions = 0;
//   auto start = high_resolution_clock::now();

//     hpp::fcl::Capsule shape1(0.1+dist(e2)*0.1, 0.2+dist(e2)*0.1);
//     hpp::fcl::Capsule shape2(0.1+dist(e2)*0.1, 0.2+dist(e2)*0.1);
//     //hpp::fcl::Sphere shape1(dist(e2));
//     //hpp::fcl::Sphere shape2(dist(e2));

//     Eigen::Matrix3d m1;
//     m1 = Eigen::AngleAxisd(dist(e2)*M_PI, Eigen::Vector3d::UnitZ())
//         * Eigen::AngleAxisd(dist(e2)*M_PI, Eigen::Vector3d::UnitY())
//         * Eigen::AngleAxisd(dist(e2)*M_PI, Eigen::Vector3d::UnitZ());
//     Eigen::Matrix3d m2;
//     m2 = Eigen::AngleAxisd(dist(e2)*M_PI, Eigen::Vector3d::UnitZ())
//         * Eigen::AngleAxisd(dist(e2)*M_PI, Eigen::Vector3d::UnitY())
//         * Eigen::AngleAxisd(dist(e2)*M_PI, Eigen::Vector3d::UnitZ());
    
//     Eigen::Vector3d v1(dist(e2),dist(e2),dist(e2));
//     Eigen::Vector3d v2(dist(e2),dist(e2),dist(e2));

//     Transform3f T1(m1, v1);
//     Transform3f T2(m2, v2);
//     CollisionRequest req;

//   for (int i=0; i<100000; i++)
//   {
//     // Compute collision
    
//     req.enable_cached_gjk_guess = false;
//     req.gjk_variant = hpp::fcl::GJKVariant::NesterovAcceleration;
//     //req.distance_upper_bound = 0.3;
//     req.security_margin = 0.2;

//     CollisionResult res;
//     collide(&shape1, T1, &shape2, T2, req, res);
//     req.updateGuess(res);

//     //T1.translation() += Eigen::Vector3d(dist(e2)*0.01,dist(e2)*0.01,dist(e2)*0.01);
//     //T2.translation() += Eigen::Vector3d(dist(e2)*0.01,dist(e2)*0.01,dist(e2)*0.01);

//     Eigen::Matrix3d m1e;
//     m1e = Eigen::AngleAxisd(dist(e2)*0.001, Eigen::Vector3d::UnitZ())
//         * Eigen::AngleAxisd(dist(e2)*0.001, Eigen::Vector3d::UnitY())
//         * Eigen::AngleAxisd(dist(e2)*0.001, Eigen::Vector3d::UnitZ());
//     T1.rotation() *= m1e;

//     Eigen::Matrix3d m2e;
//     m2e = Eigen::AngleAxisd(dist(e2)*0.001, Eigen::Vector3d::UnitZ())
//         * Eigen::AngleAxisd(dist(e2)*0.001, Eigen::Vector3d::UnitY())
//         * Eigen::AngleAxisd(dist(e2)*0.001, Eigen::Vector3d::UnitZ());
//     T2.rotation() *= m2e;

//     total_collisions += res.isCollision();

//     //ComputeCollision collide_functor(&shape1, &shape2);
//     //collide_functor(T1, T2, req, res);
//   }
//   auto stop = high_resolution_clock::now();
//   auto duration = duration_cast<microseconds>(stop - start);

//   printf("timing: %d\n", duration);
//   printf("total collisions: %d\n", total_collisions);


//   return 0;
// }


#include <ceres/ceres.h>

// Define the cost functor
struct MyCostFunctor {
  template<typename T>
  bool operator()(const T* const* params, T* residuals) const {
    // Extract the parameters
    const T* x = params[0];
    const T* y = params[1];

    // Compute residuals using x and y
    residuals[0] = T(10.0) - *x;
    residuals[1] = *y - T(20.0);
    return true;
  }
};

int main() {
  // Initialize the problem
  ceres::Problem problem;

  // Add parameters to the problem
  double x = 0.0; // initial value for x
  double y = 0.0; // initial value for y
  problem.AddParameterBlock(&x, 1); // x has 1 component
  problem.AddParameterBlock(&y, 1); // y has 1 component

  // Add the cost functor to the problem
  ceres::DynamicAutoDiffCostFunction<MyCostFunctor>* cost_function =
    new ceres::DynamicAutoDiffCostFunction<MyCostFunctor>(new MyCostFunctor);
  cost_function->AddParameterBlock(1); // x has 1 parameter
  cost_function->AddParameterBlock(1); // y has 1 parameter
  cost_function->SetNumResiduals(2); // 2 residuals
  problem.AddResidualBlock(cost_function, nullptr, &x, &y);

  // Configure solver options
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  // Solve the problem
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Print optimization results
  std::cout << summary.FullReport() << std::endl;
  std::cout << "Optimized parameters: x = " << x << ", y = " << y << std::endl;

  return 0;
}