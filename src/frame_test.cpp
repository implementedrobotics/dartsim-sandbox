#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>
#include <dart/gui/osg/TrackballManipulator.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;


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
    dart::utils::DartLoader loader;
    std::string urdf = "/home/nomad/dev/dartsim-sandbox/resource/Nomad/Nomad_leg_test.urdf";
    SkeletonPtr leg = loader.parseSkeleton(urdf);

    auto world = World::create();
    world->addSkeleton(leg);
    
    ::osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> node = new dart::gui::osg::RealTimeWorldNode(world);
    

    // Add Local Frames
    //Eigen::Vector3d mOffset = Eigen::Vector3d(0.0, 0, 0);

    // Create target Frame
    //Eigen::Isometry3d tf = leg->getBodyNode("Foot")->getTransform();
    //tf.pretranslate(mOffset);
    //std::cout << tf.translation() << std::endl;
    //mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target", tf);

    //world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("HFE_Actuator"), "hfe_link/frame"));
    //world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("KFE_Actuator"), "kfe_link/frame"));
    world->addSimpleFrame(create_interactive_frame(dart::dynamics::Frame::World(), "world/frame"));
    world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("base_link"), "base_link/frame"));
    world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("upper_Leg"), "upper_link/frame"));
    world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("lower_Leg"), "lower_link/frame"));
    world->addSimpleFrame(create_interactive_frame(leg->getBodyNode("foot"), "foot/frame"));

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

    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  world->setGravity(gravity);

    //world->setGravity([0,-10,0]);
    auto viewer = dart::gui::osg::Viewer();

    viewer.addWorldNode(node);
    viewer.setUpViewInWindow(0,0,640,480);
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(0.0f, -5.0f, 0.0f),
    ::osg::Vec3(0.0f, 0.0f, 0.0f),
    ::osg::Vec3(0.0f, 0.0f, 1.0f), false);

    viewer.setCameraManipulator(viewer.getCameraManipulator());
    std::cout << leg->getBodyNode("base_link")->getWorldTransform().translation() << std::endl;
    std::cout << leg->getDof(0)->getName() << std::endl;
    std::cout << leg->getDof(1)->getName() << std::endl;
    //std::cout << leg->getDof(2)->getName() << std::endl;
    //std::cout << leg->getDof(3)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    
    //leg->getDof(0)->setPosition(1.5);
    //leg->getDof(0)->setRestPosition(0);
    leg->getDof(0)->setDampingCoefficient(0.2);
    //leg->getDof(1)->setSpringStiffness(1000);
    //leg->getDof(1)->setDampingCoefficient(.1);//Velocity(4);

    leg->getDof(0)->setPositionLimits(-0.1,0.1);
    leg->getDof(0)->setForce(1000);
    std::cout << "DOF: " << leg->getDof(1)->getName();
    auto hip_node = leg->getBodyNode("hfe_motor");
    auto foot_node = leg->getBodyNode("foot");

    const Eigen::MatrixXd& J = leg->getJacobian(foot_node, hip_node);
    

    std::cout << J << std::endl;
    viewer.run();
    return 0;
}