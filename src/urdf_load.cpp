#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>


using namespace dart::dynamics;
using namespace dart::simulation;

int main(int argc, char *argv[])
{
    dart::utils::DartLoader loader;
    std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
    urdf.append("/NomadFull/Nomad.urdf");
   // std::string urdf = "/home/nomad/dev/dartsim-sandbox/resource/NomadFull/Nomad.urdf";
    SkeletonPtr leg = loader.parseSkeleton(urdf);
    std::cout << "LOADED!" << std::endl;

    auto world = World::create();
    world->addSkeleton(leg);
    
    ::osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> node = new dart::gui::osg::RealTimeWorldNode(world);

    auto viewer = dart::gui::osg::Viewer();
    viewer.allowSimulation(true);
    viewer.addWorldNode(node);
    viewer.setUpViewInWindow(0,0,640,480);
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(2.57f, 3.14f, 1.64f),
    ::osg::Vec3(0.0f, 0.0f, 0.0f),
    ::osg::Vec3(-0.24f, -0.25f, 0.94f), true);

    viewer.setCameraManipulator(viewer.getCameraManipulator());
    std::cout << leg->getBodyNode("base_link")->getWorldTransform().translation();

    std::cout << leg->getDof(7)->getName() << std::endl;
    std::cout << leg->getDof(8)->getName() << std::endl;
    //std::cout << leg->getDof(2)->getName() << std::endl;
    //std::cout << leg->getDof(3)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    //std::cout << leg->getDof(0)->getName() << std::endl;
    //leg->getDof(7)->setPosition(1.5);
    //leg->getDof(8)->setPosition(1.5);
    //leg->getDof(0)->setRestPosition(0);
    //leg->getDof(0)->setDampingCoefficient(0.2);
    //leg->getDof(1)->setDampingCoefficient(.1);//Velocity(4);

   // auto hip_node = leg->getBodyNode("HFE_Actuator1");
   // auto foot_node = leg->getBodyNode("Foot1");

   // const Eigen::MatrixXd& J = leg->getJacobian(foot_node, hip_node);
    

   // std::cout << J << std::endl;
    viewer.run();
    return 0;
}