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


SimpleFramePtr g_hip_target;
double g_stand_height = 0.4f;
class NomadSimWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  explicit NomadSimWorldNode(WorldPtr world, SkeletonPtr nomad)
      : dart::gui::osg::RealTimeWorldNode(world), nomad_(nomad)
  {
    // World gets saved as -> mWorld if ever needed.  Also ->getWorld()
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    ShapePtr ball = std::make_shared<SphereShape>(0.025);

    g_hip_target = std::make_shared<SimpleFrame>(Frame::World(), "target", tf);

    g_hip_target->setShape(ball);
    g_hip_target->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0, 0));
    world->addSimpleFrame(g_hip_target);
  }

  void customPreStep()
  {
    // Use this function to execute custom code before each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
    //std::cout << "Hello" << std::endl;
    // leg->getDof(2)->setForce(20.3);

    // std::cout << leg->getJoint("leg_to_world")->getPosition(0) << std::endl;

  }

  void customPostStep()
  {
    // Use this function to execute custom code after each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
  }

protected:
  SkeletonPtr nomad_;
  SimpleFramePtr hip_target_;
};

class NomadInputHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit NomadInputHandler(dart::gui::osg::Viewer *viewer, NomadSimWorldNode *world_node, const SkeletonPtr &nomad)
      : viewer_(viewer), world_node_(world_node), nomad_(nomad)
  {
    hip_target_offset = 0.0f;
    Reset();
  }

virtual bool handle(const ::osgGA::GUIEventAdapter &ea, ::osgGA::GUIActionAdapter&) override
{
  // Return if null
  if(nullptr == nomad_)
  {
    return false;
  }

  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
  {
    if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Up)
    {
      std::cout << "Arrow Key Up!" << std::endl;

      Eigen::Vector3d hipOffset = g_hip_target->getWorldTransform().translation();
      hipOffset = hipOffset + Eigen::Vector3d(0,0,0.01);
      g_hip_target->setTranslation(hipOffset);

      std::cout << g_hip_target->getWorldTransform().translation() << std::endl;

    }
    else if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Down)
    {
      std::cout << "Arrow Key Down!" << std::endl;

      Eigen::Vector3d hipOffset = g_hip_target->getWorldTransform().translation();
      hipOffset = hipOffset + Eigen::Vector3d(0,0,-0.01);

      g_hip_target->setTranslation(hipOffset);
      std::cout << g_hip_target->getWorldTransform().translation() << std::endl;
    }
    else if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_R)
    {
      std::cout << "Reset" << std::endl;
      Reset();
    }
  }
}

void Reset()
{
  // Update Hip Target
  g_hip_target->setTranslation(Eigen::Vector3d(0,-.1,g_stand_height));

  nomad_->getJoint("leg_to_world")->setPositionLimitEnforced(true);
  nomad_->getDof("leg_to_world")->setPosition(0.15);
  nomad_->getJoint("kfe_joint")->setPositionLimitEnforced(true);
  nomad_->getDof("kfe_joint")->setPositionLimits(-1.32, 1.0);

  // Set Positions
  nomad_->getDof("hfe_joint")->setPosition(1.3);
  nomad_->getDof("kfe_joint")->setPosition(-1.3);

  // Joint PD Params
  nomad_->getDof("hfe_joint")->setDampingCoefficient(0.2);
  nomad_->getDof("hfe_joint")->setSpringStiffness(10);
  nomad_->getDof("hfe_joint")->setRestPosition(0.5);

  // Joint PD Params
  nomad_->getDof("kfe_joint")->setSpringStiffness(100);
  nomad_->getDof("kfe_joint")->setRestPosition(0.7);
  nomad_->getDof("kfe_joint")->setDampingCoefficient(0.1); //Velocity(4);

}
protected:

  dart::gui::osg::Viewer *viewer_;
  NomadSimWorldNode *world_node_;
  SkeletonPtr nomad_;
  double hip_target_offset;

};

dart::gui::osg::InteractiveFramePtr
create_interactive_frame(dart::dynamics::Frame *frame, const std::string name)
{
  Eigen::Vector3d mOffset = Eigen::Vector3d(0.5, 0, 0);

  // Create target Frame
  //Eigen::Isometry3d tf = frame->getTransform();
  //tf.pretranslate(mOffset);

  dart::gui::osg::InteractiveFramePtr interactive_frame = std::make_shared<dart::gui::osg::InteractiveFrame>(frame, name);

  // Turn off unnecessary views
  for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR, dart::gui::osg::InteractiveTool::PLANAR})
    for (std::size_t i = 0; i < 3; ++i)
      interactive_frame->getTool(type, i)->setEnabled(false);

  return interactive_frame;
}

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

  return ground;

}
SkeletonPtr LoadNomad()
{
  dart::utils::DartLoader loader;
  std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
  urdf.append("/Nomad/Nomad_leg_test.urdf");

  SkeletonPtr nomad = loader.parseSkeleton(urdf);

  nomad->getJoint("leg_to_world")->setPositionLimitEnforced(true);
  nomad->getDof("leg_to_world")->setPosition(0.15);
  nomad->getJoint("kfe_joint")->setPositionLimitEnforced(true);
  nomad->getDof("kfe_joint")->setPositionLimits(-1.32, 1.0);

  // Set Positions
  nomad->getDof("hfe_joint")->setPosition(1.3);
  nomad->getDof("kfe_joint")->setPosition(-1.3);

  // Joint PD Params
  nomad->getDof("hfe_joint")->setDampingCoefficient(0.2);
  nomad->getDof("hfe_joint")->setSpringStiffness(10);
  nomad->getDof("hfe_joint")->setRestPosition(0.5);

  // Joint PD Params
  nomad->getDof("kfe_joint")->setSpringStiffness(100);
  nomad->getDof("kfe_joint")->setRestPosition(0.7);
  nomad->getDof("kfe_joint")->setDampingCoefficient(0.1); //Velocity(4);

  return nomad;

}
int main(int argc, char *argv[])
{

  // Create dart simulation world
  WorldPtr world = World::create();

  // Update Gravity vector to Z down
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  world->setGravity(gravity);

  // Create and add ground to world
  SkeletonPtr ground = CreateGround();
  world->addSkeleton(ground);
  
  // Create and add Nomad test robot to world
  SkeletonPtr nomad = LoadNomad();
  world->addSkeleton(nomad);

  // Create osg world node
  ::osg::ref_ptr<NomadSimWorldNode> node = new NomadSimWorldNode(world, nomad);

  // Create osg Viewer
  dart::gui::osg::Viewer viewer = dart::gui::osg::Viewer();
  viewer.addWorldNode(node);

  // Default with Headlights
  viewer.switchHeadlights(false);

  // Add input handler
  viewer.addEventHandler(new NomadInputHandler(&viewer, node, nomad));
  
  // Print out instructions for the viewer
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 1280, 1024);
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(0.0f, -5.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.707f, 0.707f), false);

  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();

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


/*


  std::cout << leg->getJoint(2)->getName() << " Joint" << std::endl;
  leg->getJoint(3)->setPositionLimitEnforced(true);
  std::cout << "DOF: " << leg->getDof(1)->getName();
  auto hip_node = leg->getBodyNode("hfe_motor");
  auto foot_node = leg->getBodyNode("foot");

  const Eigen::MatrixXd &J = leg->getJacobian(foot_node, hip_node);

  std::cout << J << std::endl;
*/ 
  return 0;
}