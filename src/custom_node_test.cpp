#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>
#include <dart/gui/osg/TrackballManipulator.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;

SimpleFramePtr mTarget = nullptr;
SkeletonPtr leg = nullptr;
class CustomWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  explicit CustomWorldNode(const dart::simulation::WorldPtr& world = nullptr)
    : dart::gui::osg::RealTimeWorldNode(world)
  {
    // Set up the customized WorldNode
  }

  void customPreRefresh()
  {
    // Use this function to execute custom code before each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
  }

  void customPostRefresh()
  {
    // Use this function to execute custom code after each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
  }

  void customPreStep()
  {
    // Use this function to execute custom code before each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
    //std::cout << "Hello" << std::endl;
   // leg->getDof(2)->setForce(20.3);

    
    // std::cout << leg->getJoint("leg_to_world")->getPosition(0) << std::endl;
    mTarget->setTranslation(Eigen::Vector3d(0,0,0.5));
  }

  void customPostStep()
  {
    // Use this function to execute custom code after each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
  }
};


dart::gui::osg::InteractiveFramePtr create_interactive_frame(dart::dynamics::Frame* frame, const std::string name)
{
    Eigen::Vector3d mOffset = Eigen::Vector3d(0.5, 0, 0);

    // Create target Frame
    //Eigen::Isometry3d tf = frame->getTransform();
    //tf.pretranslate(mOffset);

    dart::gui::osg::InteractiveFramePtr interactive_frame = std::make_shared<dart::gui::osg::InteractiveFrame>(frame, name);

    // Turn off unnecessary views
    for(const auto type : {dart::gui::osg::InteractiveTool::ANGULAR, dart::gui::osg::InteractiveTool::PLANAR})
        for(std::size_t i=0; i < 3; ++i)
            interactive_frame->getTool(type, i)->setEnabled(false);

    return interactive_frame;
}
int main(int argc, char *argv[])
{

  auto world = World::create();

    dart::utils::DartLoader loader;
    std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
    urdf.append("/Nomad/Nomad_leg_test.urdf");



  // Setup Ground
  SkeletonPtr ground = Skeleton::create("ground");

  // Give the ground a body
  BodyNodePtr body
      = ground->createJointAndBodyNodePair<WeldJoint>(nullptr).second;


  double thickness = 0.1;

  ShapePtr groundShape = std::make_shared<BoxShape>(Eigen::Vector3d(1,1,thickness));
  auto shapeNode = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);

  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0,0,-thickness / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  world->addSkeleton(ground);
    //std::string urdf = "/home/nomad/dev/dartsim-sandbox/resource/Nomad/Nomad_leg_test.urdf";
    leg = loader.parseSkeleton(urdf);

    
    world->addSkeleton(leg);
    
    ::osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world);
    

    // Add Local Frames
    //Eigen::Vector3d mOffset = Eigen::Vector3d(0.0, 0, 0);

    // Create target Frame
    //Eigen::Isometry3d tf = leg->getBodyNode("Foot")->getTransform();
    //tf.pretranslate(mOffset);
    //std::cout << tf.translation() << std::endl;
    //mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target", tf);

    //world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("HFE_Actuator"), "hfe_link/frame"));
    //world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("KFE_Actuator"), "kfe_link/frame"));
   // world->addSimpleFrame(create_interactive_frame(dart::dynamics::Frame::World(), "world/frame"));
   // world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("base_link"), "base_link/frame"));
   // world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("upper_Leg"), "upper_link/frame"));
   // world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("lower_Leg"), "lower_link/frame"));
   // world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("foot"), "foot/frame"));

    //dart::gui::osg::InteractiveFramePtr frame = std::make_shared<dart::gui::osg::InteractiveFrame>(Frame::World(), "foot", tf);
    //world->addSimpleFrame(frame);
    //frame = std::make_shared<dart::gui::osg::InteractiveFrame>(leg->getBodyNode("base_link"), "base_link/frame");
    
    //world->addSimpleFrame(frame);

    //for(const auto type : {dart::gui::osg::InteractiveTool::ANGULAR, dart::gui::osg::InteractiveTool::PLANAR})
    //    for(std::size_t i=0; i < 3; ++i)
    //        frame->getTool(type, i)->setEnabled(false);

    //world->addSimpleFrame(std::make_shared<dart::gui::osg::InteractiveFrame>(leg->getBodyNode("base_link"), "base_link/frame"));


/*
    for(std::size_t i = 0;i < leg->getNumBodyNodes(); ++i)
    {
        BodyNode* bn = leg->getBodyNode(i);
        std::cout << bn->getName() << std::endl;
        world->addSimpleFrame(std::make_shared<dart::gui::osg::InteractiveFrame>(bn, bn->getName()+"/frame"));

    }*/

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    
    
    //world->setGravity([0,-10,0]);
    auto viewer = dart::gui::osg::Viewer();

    viewer.addWorldNode(node);
    viewer.setUpViewInWindow(0,0,1280,1024);
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(0.0f, -5.0f, 0.0f),
    ::osg::Vec3(0.0f, 0.0f, 0.0f),
    ::osg::Vec3(0.0f, 0.707f, 0.707f), false);

    viewer.setCameraManipulator(viewer.getCameraManipulator());
    std::cout << leg->getBodyNode("base_link")->getWorldTransform().translation() << std::endl;
    std::cout << leg->getDof(0)->getName() << std::endl;
    std::cout << leg->getDof(1)->getName() << std::endl;
    //std::cout << leg->getDof(2)->getName() << std::endl;
    //std::cout << leg->getDof(3)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    
    leg->getJoint("leg_to_world")->setPositionLimitEnforced(true);
    leg->getDof(0)->setPosition(0.15);
    leg->getDof(1)->setPosition(1.3);
    leg->getDof(2)->setPosition(-1.3);
    //leg->getDof(0)->setRestPosition(0);
    leg->getDof(1)->setDampingCoefficient(100.2);
    //leg->getDof(0)->setSpringStiffness(1000);

    leg->getDof(1)->setDampingCoefficient(0.2);
    leg->getDof(1)->setSpringStiffness(10);
    leg->getDof(1)->setRestPosition(0.5);

    leg->getDof(2)->setDampingCoefficient(0.2);
    leg->getDof(2)->setSpringStiffness(100);
    leg->getDof(2)->setRestPosition(0.7);
    leg->getDof(2)->setDampingCoefficient(0.1);//Velocity(4);

    leg->getJoint("kfe_joint")->setPositionLimitEnforced(true);
    leg->getDof("kfe_joint")->setPositionLimits(-1.32, 1.0);

   // leg->getDof(0)->setPositionLimits(-0.1,0.1);
   // leg->getDof(0)->setForce(1000);

    Eigen::Isometry3d tf2(Eigen::Isometry3d::Identity());
    //tf.pretranslate(mOffset);
    mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target", tf2);
   // ShapePtr ball = std::make_shared<SphereShape>(0.025);
ShapePtr ball(new SphereShape(0.025));
    //ShapePtr ball(new SphereShape(0.025));
    mTarget->setShape(ball);
    mTarget->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0, 0));
    world->addSimpleFrame(mTarget);


    std::cout << leg->getJoint(2)->getName() << " Joint" << std::endl;
    leg->getJoint(3)->setPositionLimitEnforced(true);
    std::cout << "DOF: " << leg->getDof(1)->getName();
    auto hip_node = leg->getBodyNode("hfe_motor");
    auto foot_node = leg->getBodyNode("foot");

    const Eigen::MatrixXd& J = leg->getJacobian(foot_node, hip_node);
    

    std::cout << J << std::endl;
    viewer.run();
    return 0;
}