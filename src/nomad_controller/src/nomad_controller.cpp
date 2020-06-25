#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>
#include <dart/gui/osg/TrackballManipulator.hpp>

#include <vector>

#include <FiniteStateMachine.h>
#include <TransitionEvent.h>
#include <State.h>

#include <NomadRobot.h>

using namespace dart::dynamics;
using namespace dart::simulation;

SimpleFramePtr g_hip_target;
WorldPtr g_world;

int g_key_event;

std::shared_ptr<NomadRobot> g_nomad;

// Robot Class
// Dynamic State Vector
// Floating Base Class

class NomadSimWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  explicit NomadSimWorldNode(WorldPtr world, const std::shared_ptr<NomadRobot> nomad)
      : dart::gui::osg::RealTimeWorldNode(world), nomad_(nomad), world_(world)
  {
    // World gets saved as -> mWorld if ever needed.  Also ->getWorld()
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    ShapePtr ball = std::make_shared<SphereShape>(0.025);

    g_hip_target = std::make_shared<SimpleFrame>(Frame::World(), "target", tf);

    g_hip_target->setShape(ball);
    g_hip_target->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0, 0));
    world->addSimpleFrame(g_hip_target);

    step_iter = 0;
  }

  void customPreStep()
  {

    // std::cout << leg->getJoint("leg_to_world")->getPosition(0) << std::endl;
    if (step_iter < 5)
      return;

    // Reset Leg Controller
    nomad_->ProcessInputs();
    nomad_->Run(world_->getTimeStep());
    nomad_->SendOutputs();

    // Setup
    // Reset Command
    // g_Controller->Reset();

    // Run State Machine
    // g_FSM->Run(0.05);

    //g_Controller->SetForceFeedForward(Eigen::Vector3d(0,0,-88));

    // Run LegController
    //g_Controller->Run();

    // Post Setup
  }

  void customPostStep()
  {
    // Use this function to execute custom code after each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
    //g_Controller->UpdateState();
    nomad_->UpdateState();
    g_key_event = 0;
    step_iter++;
  }

protected:
  std::shared_ptr<NomadRobot> nomad_;
  SimpleFramePtr hip_target_;
  WorldPtr world_;
  int step_iter;
};

class NomadInputHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit NomadInputHandler(dart::gui::osg::Viewer *viewer, NomadSimWorldNode *world_node, const std::shared_ptr<NomadRobot> nomad)
      : viewer_(viewer), world_node_(world_node), nomad_(nomad)
  {
    //hip_target_offset = 0.0f;
    //Reset();
  }

  virtual bool handle(const ::osgGA::GUIEventAdapter &ea, ::osgGA::GUIActionAdapter &) override
  {
    // Return if null
    if (nullptr == nomad_)
    {
      return false;
    }
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      g_key_event = ea.getKey();

      if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Up)
      {
        std::cout << "Arrow Key Up!" << std::endl;

        //   Eigen::Vector3d hipOffset = g_hip_target->getWorldTransform().translation();
        //   hipOffset = hipOffset + Eigen::Vector3d(0, 0, 0.01);
        //   g_hip_target->setTranslation(hipOffset);

        // //  std::cout << g_hip_target->getWorldTransform().translation() << std::endl;

        // //  std::cout << g_hip_target->getTransform(nomad_->getBodyNode("base_link")).translation() << std::endl;
        // //  std::cout << "Done " << std::endl;
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Down)
      {
        // std::cout << "Arrow Key Down!" << std::endl;

        // Eigen::Vector3d hipOffset = g_hip_target->getWorldTransform().translation();
        // hipOffset = hipOffset + Eigen::Vector3d(0, 0, -0.01);

        // g_hip_target->setTranslation(hipOffset);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_R)
      {
        std::cout << "Reset" << std::endl;
        Reset();
      }
      // else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_S)
      // {
      //     std::cout << "Got Stand Request" << std::endl;
      // }
      // else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_C)
      // {
      //     std::cout << "Got Crouch Request" << std::endl;
      // }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_E)
      {
         nomad_->Skeleton()->getBodyNode("base_link")->addExtForce(Eigen::Vector3d(0,-500,0), Eigen::Vector3d::Zero(), true, true);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Q)
      {
         nomad_->Skeleton()->getBodyNode("base_link")->addExtForce(Eigen::Vector3d(0,500,0), Eigen::Vector3d::Zero(), true, true);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_W)
      {
         nomad_->Skeleton()->getBodyNode("base_link")->addExtForce(Eigen::Vector3d(-1000,0,0), Eigen::Vector3d::Zero(), true, true);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_X)
      {
         nomad_->Skeleton()->getBodyNode("base_link")->addExtForce(Eigen::Vector3d(1000,0,0), Eigen::Vector3d::Zero(), true, true);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_A)
      {
         nomad_->Skeleton()->getBodyNode("base_link")->addExtForce(Eigen::Vector3d(0,0,-5000), Eigen::Vector3d::Zero(), true, true);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_D)
      {
         nomad_->Skeleton()->getBodyNode("base_link")->addExtForce(Eigen::Vector3d(0,0, 5000), Eigen::Vector3d::Zero(), true, true);
      }

    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
    {
      g_key_event = 0;
    }
  }

  void Reset()
  {
    //SetInitialPosition(nomad_);
    //nomad_->SetInitialPose();
    //nomad_->Reset();
  }

protected:
  dart::gui::osg::Viewer *viewer_;
  NomadSimWorldNode *world_node_;
  std::shared_ptr<NomadRobot> nomad_;
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

  ShapePtr groundShape = std::make_shared<BoxShape>(Eigen::Vector3d(4, 4, thickness));
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

  // Create dart simulation world
  g_world = World::create();

  // Update Gravity vector to Z down
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  g_world->setGravity(gravity);

  // Create and add ground to world
  SkeletonPtr ground = CreateGround();
  g_world->addSkeleton(ground);

  g_nomad = std::make_shared<NomadRobot>(g_world);

  std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
  urdf.append("/NomadFull/Nomad.urdf");

  g_nomad->LoadFromURDF(urdf);
  g_nomad->SetInitialPose();
  g_nomad->CreateLegControllers();
  g_nomad->SetKeyEvent(&g_key_event);

  // Create osg world node
  ::osg::ref_ptr<NomadSimWorldNode> node = new NomadSimWorldNode(g_world, g_nomad);

  // Create osg Viewer
  dart::gui::osg::Viewer viewer = dart::gui::osg::Viewer();
  viewer.addWorldNode(node);

  // Default with Headlights
  viewer.switchHeadlights(false);

  //Add Grid Attachment
  ::osg::ref_ptr<dart::gui::osg::GridVisual> grid = new dart::gui::osg::GridVisual();

  viewer.addAttachment(grid);
  // Add input handler
  viewer.addEventHandler(new NomadInputHandler(&viewer, node, g_nomad));

  // Print out instructions for the viewer
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 1280, 1024);
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(0.0f, -5.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.707f, 0.707f), false);

  viewer.setCameraManipulator(viewer.getCameraManipulator());

  //world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("HFE_Actuator"), "hfe_link/frame"));
  //world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("KFE_Actuator"), "kfe_link/frame"));
  //g_world->addSimpleFrame(create_interactive_frame(dart::dynamics::Frame::World(), "world/frame"));
  //g_world->addSimpleFrame(create_interactive_frame(nomad->getBodyNode("base_link"), "base_link/frame"));
  //world->addSimpleFrame(create_interactive_frame(nomad->getBodyNode("upper_Leg"), "upper_link/frame"));
  //world->addSimpleFrame(create_interactive_frame(nomad->getBodyNode("lower_Leg"), "lower_link/frame"));

  viewer.run();

  return 0;
}