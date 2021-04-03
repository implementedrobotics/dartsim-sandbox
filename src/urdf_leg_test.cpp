#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>


float q1_ref = -1.2f;
float q2_ref = -0.1f;
float K_p = 50.0f;
float K_d = 2.8f;
using namespace dart::dynamics;
using namespace dart::simulation;
SkeletonPtr leg;

// Using the Eigen library, using the SVD decomposition method to solve the matrix pseudo-inverse, the default error er is 0
Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd & origin, const float er = 0) {
    // perform svd decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin,
                                                 Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
    // Build SVD decomposition results
    Eigen::MatrixXd U = svd_holder.matrixU();
    Eigen::MatrixXd V = svd_holder.matrixV();
    Eigen::MatrixXd D = svd_holder.singularValues();

    // Build the S matrix
    Eigen::MatrixXd S(V.cols(), U.cols());
    S.setZero();

    //Eigen::MatrixXd S = Eigen::MatrixXd::Zero(V.cols(), U.cols());//    (V.cols(), U.cols());
    for (unsigned int i = 0; i < D.size(); ++i) {

        if (D(i, 0) > er) {
            S(i, i) = 1 / D(i, 0);
        } else {
            S(i, i) = 0;
        }
    }

    // pinv_matrix = V * S * U^T
    return V * S * U.transpose();
}


class NomadSimWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  explicit NomadSimWorldNode(WorldPtr world)
      : dart::gui::osg::RealTimeWorldNode(world), world_(world)
  {
    // // World gets saved as -> mWorld if ever needed.  Also ->getWorld()
    // Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    // ShapePtr ball = std::make_shared<SphereShape>(0.025);

    // g_hip_target = std::make_shared<SimpleFrame>(Frame::World(), "target", tf);

    // g_hip_target->setShape(ball);
    // g_hip_target->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0, 0));
    // world->addSimpleFrame(g_hip_target);

    step_iter = 0;
  }

  void customPreStep()
  {

    // std::cout << leg->getJoint("leg_to_world")->getPosition(0) << std::endl;
    if (step_iter < 5)
      return;

    // Compute PD Torques

    float q1_pos = leg->getDof(6)->getPosition();
    float q2_pos = leg->getDof(7)->getPosition();
    float q1_vel = leg->getDof(6)->getVelocity();
    float q2_vel = leg->getDof(7)->getVelocity();

    float q1_tau = K_p * (q1_ref - q1_pos) + K_d * (0.0f - q1_vel);
    float q2_tau = K_p * (q2_ref - q2_pos) + K_d * (0.0f - q2_vel);


    leg->getDof(6)->setForce(q1_tau);
    leg->getDof(7)->setForce(q2_tau);

    // Reset Leg Controller
   // nomad_->ProcessInputs();
   // nomad_->Run(world_->getTimeStep());
    //nomad_->SendOutputs();

    // Setup
    // Reset Command
    // g_Controller->Reset();

    // Run State Machine
    // g_FSM->Run(0.05);

    //g_Controller->SetForceFeedForward(Eigen::Vector3d(0,0,-88));

    // Run LegController
    //g_Controller->Run();

    // Post Setup
//    leg->getDof(7)->setForce(1);

    //std::cout << "Print!" << std::endl;
  }

  void customPostStep()
  {
    // Use this function to execute custom code after each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
    //g_Controller->UpdateState();
    //nomad_->UpdateState();
    //g_key_event = 0;
    step_iter++;

    // if(step_iter > 1000)
    // {
    //   q1_ref = -0.0f;
    //   q2_ref = -2.0f;
    //   K_p = 20.0f;
    //   K_d = 0.2f;
    //   std::cout << "Jumping!" << std::endl;
    // }
  }

protected:
  //std::shared_ptr<NomadRobot> nomad_;
 // SimpleFramePtr hip_target_;
  WorldPtr world_;
  int step_iter;
};

SkeletonPtr CreateGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  // Give the ground a body
  BodyNodePtr body = ground->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  double thickness = 0.1;

  ShapePtr groundShape = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, thickness));
  auto shapeNode = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);

  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0, 0, -thickness / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  //shapeNode->getDynamicsAspect()->setFrictionCoeff(10.0);
  //std::cout << "Mu: " << shapeNode->getDynamicsAspect()->getFrictionCoeff() << std::endl;

  return ground;
}

int main(int argc, char *argv[])
{
    dart::utils::DartLoader loader;
    std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
    urdf.append("/LegTest/Leg.urdf");
   // std::string urdf = "/home/nomad/dev/dartsim-sandbox/resource/NomadFull/Nomad.urdf";
    leg = loader.parseSkeleton(urdf);
    std::cout << "LOADED!" << std::endl;
     auto world = World::create();
    

   

    // Create and add ground to world
    SkeletonPtr ground = CreateGround();
    world->addSkeleton(ground);

    world->addSkeleton(leg);
    
    ::osg::ref_ptr<NomadSimWorldNode> node = new NomadSimWorldNode(world);
    //::osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> node = new dart::gui::osg::RealTimeWorldNode(world);

    auto viewer = dart::gui::osg::Viewer();
    viewer.allowSimulation(true);
    viewer.addWorldNode(node);
    viewer.setUpViewInWindow(0,0,1280,1024);
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(5.0f, 0.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.0f, 1.0f), false);

    viewer.setCameraManipulator(viewer.getCameraManipulator());

    //Add Grid Attachment
    ::osg::ref_ptr<dart::gui::osg::GridVisual> grid = new dart::gui::osg::GridVisual();

    viewer.addAttachment(grid);
    //std::cout << leg->getBodyNode("base_link")->getWorldTransform().translation();
    //std::cout << leg->getDof(7)->getName() << std::endl;
    //std::cout << leg->getDof(8)->getName() << std::endl;
    //std::cout << leg->getDof(2)->getName() << std::endl;
    //std::cout << leg->getDof(3)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    //leg->getDof(7)->setPosition(1.5);
    //leg->getDof(8)->setPosition(1.5);
    //leg->getDof(0)->setRestPosition(0);
    //leg->getDof(0)->setDampingCoefficient(0.2);
    //leg->getDof(1)->setDampingCoefficient(.1);//Velocity(4);
    //std::cout << leg->getDof(6)->getName() << std::endl;
std::cout << leg->getDof(6)->getName() << std::endl;
    std::cout << leg->getDof(7)->getName() << std::endl;

   // leg->getDof(6)->setPosition(-.1);
    //leg->getDof(7)->setSpringStiffness(10);
    //leg->getDof(7)->setDampingCoefficient(.8);
    leg->getDof(6)->setPosition(-1.2);
    //leg->getDof(7)->setRestPosition(-0.6);
    //leg->getDof(8)->setSpringStiffness(10);
    //leg->getDof(8)->setDampingCoefficient(.8);
    leg->getDof(7)->setPosition(-0.1);
    //leg->getDof(8)->setRestPosition(-1.5);

    std::cout << "Mass: " << leg->getMass() << std::endl;
    // leg->getDof(7)->setForce(100);
    //  leg->getDof(8)->setForce(-100);
   // auto hip_node = leg->getBodyNode("HFE_Actuator1");
   // auto foot_node = leg->getBodyNode("Foot1");

   // const Eigen::MatrixXd& J = leg->getJacobian(foot_node, hip_node);
    

   // std::cout << J << std::endl;
    viewer.run();
    return 0;
}