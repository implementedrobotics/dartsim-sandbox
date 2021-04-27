#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>

#include <cmath>


// C System Files

// C++ System Files

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
namespace Common
{
    class CubicPolynomialTrajectory
    {

    public:
        CubicPolynomialTrajectory(double q_f, double t_f);
        CubicPolynomialTrajectory(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f);
        CubicPolynomialTrajectory(); // Empty Trajectory

        void Generate(double q_f, double t_f);
        void Generate(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f);

        // TODO: Check for valid t between 0<->t_f
        double Position(double t);
        double Velocity(double t);
        double Acceleration(double t);

    protected:
        void ComputeCoeffs();

        Eigen::Vector4d a_; // Coefficients

        double q_0_;
        double v_0_;
        double t_0_;

        double q_f_;
        double v_f_;
        double t_f_;
    };
} // namespace Common

namespace Common
{
    CubicPolynomialTrajectory::CubicPolynomialTrajectory(double q_f, double t_f)
        : q_0_(0.0), q_f_(q_f), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(t_f)
    {
        // Compute Coefficients
        ComputeCoeffs();
    }
    CubicPolynomialTrajectory::CubicPolynomialTrajectory(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
        : q_0_(q_0), q_f_(q_f), v_0_(v_0), v_f_(v_f), t_0_(t_0), t_f_(t_f)
    {
        // Compute Coefficients
        ComputeCoeffs();
    }

    CubicPolynomialTrajectory::CubicPolynomialTrajectory() // Empty Trajectory
        : q_0_(0.0), q_f_(0.0), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(0.0)
    {
        // Compute Coefficients
        ComputeCoeffs();
    }

    void CubicPolynomialTrajectory::Generate(double q_f, double t_f)
    {
        q_0_ = 0.0;
        q_f_ = q_f;
        v_0_ = 0.0;
        v_f_ = 0.0;
        t_0_ = 0.0;
        t_f_ = t_f;

        ComputeCoeffs();
    }

    void CubicPolynomialTrajectory::Generate(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
    {
        q_0_ = q_0;
        q_f_ = q_f;
        v_0_ = v_0;
        v_f_ = v_f;
        t_0_ = t_0;
        t_f_ = t_f;

        ComputeCoeffs();
    }

    // TODO: Check for valid t between 0<->t_f
    double CubicPolynomialTrajectory::Position(double t)
    {
        // Trim Position
        // TODO: Function for this trimming/mapping
        double t_eval = std::min(t, t_f_);
        t_eval = std::max(t_eval, t_0_);
        return a_(0) + a_(1) * t_eval + a_(2) * t_eval * t_eval + a_(3) * t_eval * t_eval * t_eval;
    }
    double CubicPolynomialTrajectory::Velocity(double t)
    {
        // Trim Position
        // TODO: Function for this trimming/mapping
        double t_eval = std::min(t, t_f_);
        t_eval = std::max(t_eval, t_0_);
        return a_(1) + 2 * a_(2) * t_eval + 3 * a_(3) * t_eval * t_eval;
    }
    double CubicPolynomialTrajectory::Acceleration(double t)
    {
        // Trim Position
        // TODO: Function for this trimming/mapping
        double t_eval = std::min(t, t_f_);
        t_eval = std::max(t_eval, t_0_);
        return 2 * a_(2) + 6 * a_(3) * t_eval;
    }

    void CubicPolynomialTrajectory::ComputeCoeffs()
    {

        Eigen::Matrix4d C; // Constraints
        C << 1, t_0_, t_0_ * t_0_, t_0_ * t_0_ * t_0_,
            0, 1, 2 * t_0_, 3 * t_0_ * t_0_,
            1, t_f_, t_f_ * t_f_, t_f_ * t_f_ * t_f_,
            0, 1, 2 * t_f_, 3 * t_f_ * t_f_;

        Eigen::Vector4d b;
        b << q_0_, v_0_, q_f_, v_f_;

        // Solve for Coefficients
        a_ = C.lu().solve(b);
    }
} // namespace Common



 Common::CubicPolynomialTrajectory com_traj_;

using namespace dart::dynamics;
using namespace dart::simulation;

float q1_ref = 0.6f;
float q2_ref = -0.7f;
float K_p = 50.0f;
float K_d = 2.8f;
float frequency = 1.0f;
float diameter = 0.05f;
double world_time = 0.0f;
Eigen::Vector2d foot_pos_des_;
Eigen::Vector2d foot_vel_des_;

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
  explicit NomadSimWorldNode(WorldPtr world,
  dart::dynamics::SkeletonPtr leg)
      : dart::gui::osg::RealTimeWorldNode(std::move(world)), leg_(std::move(leg))
  {
    // Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    // ShapePtr ball = std::make_shared<SphereShape>(0.025);

    // g_hip_target = std::make_shared<SimpleFrame>(Frame::World(), "target", tf);

    // g_hip_target->setShape(ball);
    // g_hip_target->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0, 0));
    // world->addSimpleFrame(g_hip_target);

    // hfe = leg_->getDof("j_hfe");
    // kfe = leg_->getDof("j_kfe");

    // hip_body_ = leg_->getBodyNode("HFE_Actuator1");
    // foot_body_ = leg_->getBodyNode("Foot1");

    // // Fix our base link (the stand)
    // leg_->getRootBodyNode()->moveTo<dart::dynamics::WeldJoint>(nullptr);

    // Eigen::Vector2d foot_pos_des_ = Eigen::Vector2d::Zero();
    // Eigen::Vector2d foot_vel_des_ = Eigen::Vector2d::Zero();
    step_iter = 0;
  }

  void customPreStep()
  {

    return;
    float home = 0.0308f;
    float home2 = -0.312458f;
    // Test Foot Position
    J_ = leg_->getLinearJacobian(foot_body_, hip_body_);
    J_ = J_.bottomRows(2);

    foot_pos_ = foot_body_->getTransform(hip_body_).translation().tail(2);
    foot_vel_ = (J_ * leg_->getVelocities()).tail(2);

    if(step_iter == 1)
    {
      com_traj_.Generate(foot_pos_[1], -0.35f, 0.0, 0.0, 0.0, 3.0f);
      std::cout << "POS: " << foot_pos_ << std::endl;
    }
    if (step_iter < 5)
    {
      foot_pos_des_ = foot_pos_;

      std::cout << "J Size: " << J_.rows() << " : " << J_.cols() << std::endl;
      std::cout << "VEL: " << leg_->getVelocities().size() << std::endl;
      return;
    }

        if (step_iter > 1500)
    {
    double foot_z_pos = com_traj_.Position(world_time);
    double foot_z_vel = com_traj_.Velocity(world_time);

    foot_pos_des_[0] = 0.0f;
    foot_pos_des_[1] = foot_z_pos;
    foot_vel_des_[1] = foot_z_vel;

    std::cout << "DESIRED: " << foot_z_pos << " : " << foot_pos_[1] << std::endl;

    }

   //std::cout << "Foot Pos: " << foot_pos_[0] << ", " << foot_pos_[1] << std::endl;


    world_time += mWorld->getTimeStep();


  //  std::cout << "YEP" << std::endl;


   // foot_pos_des_[0] = diameter * std::sin(2 * M_PI * frequency * world_time);


   // std::cout << "YEP2" << std::endl;
    //foot_pos_des_[0] = -0.0;//diameter * std::cos(2 * M_PI * frequency * world_time);
    //foot_pos_des_[1] = -0.313718;//-.4;//diameter * std::sin(2 * M_PI * frequency * world_time);// - 0.2f;
    
    //foot_pos_des_[0] = home + diameter * std::cos(2 * M_PI * frequency * world_time);
    //foot_pos_des_[1] = home2 + diameter * std::sin(2 * M_PI * frequency * world_time);
    
    //std::cout << "DES: " << foot_pos_des_[0] << std::endl;
    

    //std::cout << "Knee DES: " << foot_pos_des_[1] << std::endl;
    //std::cout << "Knee POS: " << foot_pos_[1] << std::endl;

    //std::cout << "E: " << (foot_pos_des_ - foot_pos_) << std::endl;

    Eigen::Vector2d force_output = Eigen::Vector2d::Zero();
    Eigen::Vector2d tau_output;

    Eigen::Vector2d k_P(3000, 3000);
    Eigen::Vector2d k_D(200, 200);

    k_P_cartesian_ = k_P.asDiagonal();
    k_D_cartesian_ = k_D.asDiagonal();



    //tau_output = tau_feedforward_;
    //force_output = force_feedforward_;
    force_output += k_P_cartesian_ * (foot_pos_des_ - foot_pos_);
    force_output += k_D_cartesian_ * (foot_vel_des_ - foot_vel_);

    Eigen::VectorXd test = J_.transpose() * force_output;// + leg_->getGravityForces();

    test[0] = 0.0f;

    //test[1] = 0.0f;
    //test[2] = 0.0f;
    //std::cout << "Force: " << "X: " << force_output[0] << " Z: " << force_output[1] << std::endl;
   // std::cout << "Torque: " << "hip: " << test[0] << " knee: " << test[1] << std::endl;
    // std::cout << leg_skeleton_->getGravityForces() << std::endl;


   //  std::cout  << "Foot Position: " << foot_pos_ << std::endl;
    // Convert to Joint Torques

    //tau_output += (J_.transpose() * force_output).tail(3);
    
    // Compute PD Torques
   // float q1_pos = hfe->getPosition();
    //float q2_pos = kfe->getPosition();
    //float q1_vel = hfe->getVelocity();
    //float q2_vel = kfe->getVelocity();

    //float q1_tau = K_p * (q1_ref - q1_pos) + K_d * (0.0f - q1_vel);
    //float q2_tau = K_p * (q2_ref - q2_pos) + K_d * (0.0f - q2_vel);

    leg_->setForces(test);

    //std::cout << "TAU: " << std::endl << test << std::endl;
    // hfe->setForce(tau_output.tail(2)[0]);
    // kfe->setForce(tau_output.tail(2)[1]);

    //leg_->setForces(tau_output.tail(2));
    
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
  }

protected:
  dart::dynamics::SkeletonPtr leg_;
  dart::dynamics::DegreeOfFreedomPtr hfe;
  dart::dynamics::DegreeOfFreedomPtr kfe;

  dart::dynamics::BodyNodePtr hip_body_;
  dart::dynamics::BodyNodePtr foot_body_;

  Eigen::Vector2d foot_pos_;
  Eigen::Vector2d foot_vel_;

  Eigen::MatrixXd J_; // Leg Jacobian

  Eigen::Matrix2d k_P_cartesian_;
  Eigen::Matrix2d k_D_cartesian_;

  // SimpleFramePtr hip_target_;
  int step_iter;
};

dart::gui::osg::InteractiveFramePtr
CreateFrameVisual(dart::dynamics::Frame *frame, const std::string name, const Eigen::Isometry3d& relativeTransform = Eigen::Isometry3d::Identity())
{
  Eigen::Vector3d mOffset = Eigen::Vector3d(0.5, 0, 0);

  // Create target Frame
  //Eigen::Isometry3d tf = frame->getTransform();
  //tf.pretranslate(mOffset);

  dart::gui::osg::InteractiveFramePtr interactive_frame = std::make_shared<dart::gui::osg::InteractiveFrame>(frame, name, relativeTransform);

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

  //shapeNode->getDynamicsAspect()->setFrictionCoeff(10.0);
  //std::cout << "Mu: " << shapeNode->getDynamicsAspect()->getFrictionCoeff() << std::endl;

  return ground;
}

SkeletonPtr LoadNomad()
{
  dart::utils::DartLoader loader;
  std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
  urdf.append("/Robot_V2/NOMAD.urdf");

  SkeletonPtr nomad = loader.parseSkeleton(urdf);

    nomad->getDof("j_haa_FL")->setPosition(-0.75f);
    nomad->getDof("j_haa_FR")->setPosition(0.75f);
    nomad->getDof("j_haa_RL")->setPosition(0.75f);
    nomad->getDof("j_haa_RR")->setPosition(-0.75f);

    nomad->getJoint("j_kfe_FL")->setPositionLimitEnforced(true);
    nomad->getJoint("j_kfe_FR")->setPositionLimitEnforced(true);
    nomad->getJoint("j_kfe_RL")->setPositionLimitEnforced(true);
    nomad->getJoint("j_kfe_RR")->setPositionLimitEnforced(true);

    nomad->getDof("j_kfe_FL")->setPositionLimits(0.0, -2.35);
    nomad->getDof("j_kfe_FR")->setPositionLimits(0.0, -2.35);
    nomad->getDof("j_kfe_RL")->setPositionLimits(0.0, -2.35);
    nomad->getDof("j_kfe_RR")->setPositionLimits(0.0, -2.35);

  // nomad->getDof("j_haa_FL")->setPosition(-0.5f);
  // nomad->getDof("j_haa_FR")->setPosition(0.5f);
  // nomad->getDof("j_haa_RL")->setPosition(0.5f);
  // nomad->getDof("j_haa_RR")->setPosition(-0.5f);
  //nomad->getDof("j_kfe")->setPosition(0.0f);

  //nomad->getJoint("j_kfe")->setPositionLimitEnforced(true);
  //nomad->getDof("j_kfe")->setDampingCoefficient(0.1f);
  //nomad->getDof("j_hfe")->setDampingCoefficient(0.1f);
  //nomad->getDof("j_kfe")->setPositionLimits(0.0f, 2.0);

  //nomad->getDof("slider")->setPositionLimits(-0.3, 0.0);

  //std::cout << nomad->getDof("slider")->getIndexInSkeleton() << std::endl;
  //nomad->getJoint("slider")->setPositionLimitEnforced(true);

  // Set Friction
  //std::cout << nomad->getBodyNode("foot")->getFrictionCoeff() << std::endl;
  //nomad->getBodyNode("foot")->setFrictionCoeff(10.8);
  return nomad;
}

int main(int argc, char *argv[])
{
  auto world = World::create();

  // Set Gravity
  world->setGravity(Eigen::Vector3d(0,0,-10.0));

  // Load the leg skeleton
  SkeletonPtr nomad = LoadNomad();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  double x = 0.0;
  double y = 0.0;
  double z = 0.75;

  // TODO: Function/Wrap This to set floating base state
  Eigen::Matrix3d orientation;
  orientation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  Eigen::Isometry3d tf;
  tf.linear() = orientation;
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.15);

  Eigen::VectorXd pos = dart::dynamics::FreeJoint::convertToPositions(tf);

  nomad->getRootJoint()->setPositions(pos); // Floating Base Position

  // Create and add ground to world
  SkeletonPtr ground = CreateGround();

  // Add Skeletons to the world
  world->addSkeleton(ground);
  world->addSkeleton(nomad);

  // Add Frames
  //const Eigen::Isometry3d offset = nomad->getJoint("j_kfe_FR")->getTransformFromChildBodyNode();
  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("b_hfe_motor_FR1"), "hfe_link_FR/frame"));
  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("b_hfe_motor_FL1"), "hfe_link_FL/frame"));
  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("b_hfe_motor_RR1"), "hfe_link_RR/frame"));
  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("b_hfe_motor_RL1"), "hfe_link_RL/frame"));

  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("b_lower_leg_FR1"), "kfe_link_FR/frame", offset));
  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("j_kfe_FL"), "kfe_link_FL/frame"));
  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("j_kfe_RR"), "kfe_link_RR/frame"));
  //world->addSimpleFrame(CreateFrameVisual(nomad->getBodyNode("j_kfe_RL"), "kfe_link_RL/frame"));

  //world->addSimpleFrame(CreateFrameVisual(leg->getBodyNode("Lower_Leg1"), "kfe_link/frame", offset));

  
  //std::cout << leg->getDof(5)->getName() << std::endl;
  //leg->getDof(5)->setPosition(0.08); // Raise the stand up a bit
  //world->addSimpleFrame(CreateFrameVisual(leg->getBodyNode("Foot1"), "foot/frame"));
  //world->addSimpleFrame(CreateFrameVisual(dart::dynamics::Frame::World(), "world/frame"));


  // Create our custom Sim Node
  ::osg::ref_ptr<NomadSimWorldNode> node = new NomadSimWorldNode(world, nomad);

  // Setup Viewer
  auto viewer = dart::gui::osg::Viewer();
  viewer.allowSimulation(true);
  viewer.addWorldNode(node);
  viewer.setUpViewInWindow(0, 0, 1280, 1024);
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(5.0f, 0.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.0f, 0.0f),
                                                 ::osg::Vec3(0.0f, 0.0f, 1.0f), false);

  viewer.setCameraManipulator(viewer.getCameraManipulator());

  //Add Grid Attachment
  ::osg::ref_ptr<dart::gui::osg::GridVisual> grid = new dart::gui::osg::GridVisual();

  viewer.addAttachment(grid);

  viewer.run();
  return 0;
}