#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/gui/osg/osg.hpp>
#include <dart/gui/osg/TrackballManipulator.hpp>

#include <vector>

using namespace dart::dynamics;
using namespace dart::simulation;

SimpleFramePtr g_hip_target;
WorldPtr g_world;
SkeletonPtr g_nomad;
double g_stand_height = 0.4f;


double max_torque = 0.0;

// Robot Class
// Dynamic State Vector
// Floating Base Class



class CubicPolynomialTrajectory
{
public:
  CubicPolynomialTrajectory(double q_f, double t_f)
      : q_0_(0.0), q_f_(q_f), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(t_f)
  {
    // Compute Coefficients
    ComputeCoeffs();
  }
  CubicPolynomialTrajectory(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
      : q_0_(q_0), q_f_(q_f), v_0_(v_0), v_f_(v_f), t_0_(t_0), t_f_(t_f)
  {
    // Compute Coefficients
    ComputeCoeffs();
  }

  CubicPolynomialTrajectory() // Empty Trajectory
      : q_0_(0.0), q_f_(0.0), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(0.0)
  {
    // Compute Coefficients
    ComputeCoeffs();
  }

  void Generate(double q_f, double t_f)
  {
    q_0_ = 0.0;
    q_f_ = q_f;
    v_0_ = 0.0;
    v_f_ = 0.0;
    t_0_ = 0.0;
    t_f_ = t_f;

    ComputeCoeffs();
  }

  void Generate(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
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
  double Position(double t)
  {
    // Trim Position
    // TODO: Function for this trimming/mapping
    double t_eval = std::min(t, t_f_);
    t_eval = std::max(t_eval, t_0_);
    return a_(0) + a_(1) * t_eval + a_(2) * t_eval * t_eval + a_(3) * t_eval * t_eval * t_eval;
  }
  double Velocity(double t)
  {
    // Trim Position
    // TODO: Function for this trimming/mapping
    double t_eval = std::min(t, t_f_);
    t_eval = std::max(t_eval, t_0_);
    return a_(1) + 2 * a_(2) * t_eval + 3 * a_(3) * t_eval * t_eval;
  }
  double Acceleration(double t)
  {
    // Trim Position
    // TODO: Function for this trimming/mapping
    double t_eval = std::min(t, t_f_);
    t_eval = std::max(t_eval, t_0_);
    return 2 * a_(2) + 6 * a_(3) * t_eval;
  }

protected:
  void ComputeCoeffs()
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

  Eigen::Vector4d a_; // Coefficients

  double q_0_;
  double v_0_;
  double t_0_;

  double q_f_;
  double v_f_;
  double t_f_;
};

class LegController
{
    public:
        LegController(SkeletonPtr &legSkeleton) : leg_skeleton_(legSkeleton)
        {
            Reset();
            hfe_body_ = leg_skeleton_->getBodyNode("hfe_motor");
            foot_body_ = leg_skeleton_->getBodyNode("foot");
        }
        void Reset()
        {
            k_P_cartesian_ = Eigen::Matrix3d::Zero();
            k_D_cartesian_ = Eigen::Matrix3d::Zero();

            k_P_joint_ = Eigen::Matrix3d::Zero();
            k_D_joint_ = Eigen::Matrix3d::Zero();

            foot_pos_desired_ = Eigen::Vector3d::Zero();
            foot_vel_desired_ = Eigen::Vector3d::Zero();

            q_desired_ = Eigen::Vector3d::Zero();
            qd_desired_ = Eigen::Vector3d::Zero();
            tau_feedforward_ = Eigen::Vector3d::Zero();

            force_feedforward_ = Eigen::Vector3d::Zero();
        }

        void SetCartesianPD(const Eigen::Vector3d& k_P, const Eigen::Vector3d& k_D)
        {
            k_P_cartesian_ = k_P.asDiagonal();
            k_D_cartesian_ = k_D.asDiagonal();
        }

        void SetJointPD(const Eigen::Vector3d& k_P, const Eigen::Vector3d& k_D)
        {
            k_P_joint_ = k_P.asDiagonal();
            k_D_joint_ = k_D.asDiagonal();
        }

        void SetFootStateDesired(const Eigen::Vector3d &posDesired, const Eigen::Vector3d &velDesired)
        {
            foot_pos_desired_ = posDesired;
            foot_vel_desired_ = velDesired;
        }
        void SetForceFeedForward(const Eigen::Vector3d &force)
        {
            force_feedforward_ = force;
        }
        const Eigen::Vector3d& GetFootPosition()
        {
          return foot_pos_;
        }
        void Run()
        {
            // Compute Feed Forwards
            Eigen::Vector3d tau_output;
            Eigen::Vector3d force_output;

            tau_output = tau_feedforward_;
            force_output = force_feedforward_;
            force_output += k_P_cartesian_ * (foot_pos_desired_ - foot_pos_);
            force_output += k_D_cartesian_ * (foot_vel_desired_ - foot_vel_);
           // std::cout << leg_skeleton_->getGravityForces() << std::endl;

            // Convert to Joint Torques
            tau_output += J_.transpose() * force_output;

            //std::cout << tau_output << std::endl;
            //max_torque = std::max(max_torque, tau_output[2]);
            //std::cout << max_torque << std::endl;
            // Technically sets a force on the unactuated joint = 0.  Just be aware. I think we are just lucky it happens
            // to match 3dof even though this leg setup is 2dof
            leg_skeleton_->setForces(tau_output);
        }

        void UpdateState()
        {
            // Compute Jacobian (Linear)  TODO: Do we want the full Jacobian with angular velocities?
            J_ = leg_skeleton_->getLinearJacobian(foot_body_, hfe_body_);

            foot_pos_ = foot_body_->getTransform(hfe_body_).translation();
            foot_vel_ = J_ * leg_skeleton_->getVelocities();

            // This is the same, but like using the jacobian better as it is more generic
            // TODO: See if linearvelocity is faster/cached for some reason
            //foot_vel_ = foot_body_->getLinearVelocity(hfe_body_, hfe_body_);
        }

        const SkeletonPtr& Skeleton() 
        {
          return leg_skeleton_;
        }
    protected:
        SkeletonPtr leg_skeleton_;  
        BodyNodePtr hfe_body_;
        BodyNodePtr foot_body_;

        Eigen::Matrix3d  k_P_joint_;
        Eigen::Matrix3d  k_D_joint_;
        Eigen::Matrix3d  k_P_cartesian_;
        Eigen::Matrix3d  k_D_cartesian_;

        Eigen::Vector3d foot_pos_desired_;
        Eigen::Vector3d foot_vel_desired_;

        Eigen::Vector3d foot_pos_;
        Eigen::Vector3d foot_vel_;
        
        Eigen::Vector3d q_desired_; // 3Dof Leg
        Eigen::Vector3d qd_desired_; // 3Dof Leg

        Eigen::Vector3d q_;
        Eigen::Vector3d qd_;

        Eigen::Vector3d force_feedforward_;
        Eigen::Vector3d tau_feedforward_;

        Eigen::MatrixXd J_; // Leg Jacobian
};

LegController *g_Controller;

class FiniteStateMachine;

FiniteStateMachine *g_FSM;
typedef enum {
  STATE_CROUCH = 1,
  STATE_STAND = 2,
  STATE_IDLE = 3,
  STATE_ESTOP = 4
} ControllerState;

class State
{
  friend class FiniteStateMachine;

public:
  State(const std::string &name, ControllerState id) : name_(name), id_(id), in_transition_(false), next_state_(this)
  {
  }
  const std::string &Name() { return name_; } // State Name
  ControllerState &Id() { return id_; }       // State ID
  virtual void Setup(){};                     // Default Do Nothing
  virtual void Enter(){};                     // Default Do Nothing
  virtual void Exit(){};                      // Default Do Nothing
  //virtual void ProcessEvent(){};                // External Event
  virtual bool Transition(State *pNextState) = 0; // Force Transition
  bool InTransition() { return in_transition_; }
  virtual void Run() = 0; // Override for state execution logic

  State *NextState() { return next_state_; } // Next State to transition to
  // TODO: Valid state transition?

protected:
  std::string name_;           // State Name
  ControllerState id_;         // State ID
  bool in_transition_;         // In Transition
  State *next_state_;          // Next State
  FiniteStateMachine *parent_; // Parent FSM that state is in
};

class CrouchState : public State
{
  public:
  CrouchState() : State("CROUCH", ControllerState::STATE_CROUCH)
  {

  }
  void Run() 
  {
      double time_now = g_world->getTime();
      double h_t = crouch_traj_.Position(time_now - start_time_);

      //std::cout << "H: " << h_t << "to: " << g_Controller->GetFootPosition()[2] << std::endl;
      Eigen::Vector3d foot_pos_desired = start_pos_;
      foot_pos_desired[2] = h_t;
      
      g_Controller->SetCartesianPD(Eigen::Vector3d(3000,3000,3000), Eigen::Vector3d(200,200,200));
      g_Controller->SetFootStateDesired(foot_pos_desired, Eigen::Vector3d::Zero());
  }
  void Enter()
   { 
    next_state_ = this;

    // Cache Current Position
    start_pos_ = g_Controller->GetFootPosition();

    std::cout << "POS!: "<< start_pos_ << std::endl;
    // Compute Trajectory from Initial Foot to Stand Height
    double crouch_height = 0.1;
    crouch_traj_.Generate(start_pos_[2], -crouch_height, 0.0, 0.0, 0.0, 0.5);
    start_time_ = g_world->getTime();

    }
  bool Transition(State *pNextState)
  {
    switch (pNextState->Id())
    {
    case ControllerState::STATE_CROUCH:
      std::cout << "Transition from Crouch to Crouch VALID" << std::endl;
      break;
    case ControllerState::STATE_STAND:
      std::cout << "Transition from Crouch to Stand VALID" << std::endl;
      next_state_ = pNextState;
      break;
    default:
      std::cout << "Invalid State Transition." << std::endl;
    }
  }

  protected:
    CubicPolynomialTrajectory crouch_traj_;
    double start_time_;
    Eigen::Vector3d start_pos_;
};

class StandState : public State
{
  public:
  StandState() : State("STAND", ControllerState::STATE_STAND)
  {

  }
  void Run()
  {
      double time_now = g_world->getTime();
      double h_t = stand_traj_.Position(time_now - start_time_);
      double a_t = stand_traj_.Acceleration(time_now - start_time_);

      //std::cout << "H: " << h_t << "to: " << g_Controller->GetFootPosition()[2] << std::endl;
      Eigen::Vector3d foot_pos_desired = start_pos_;
      foot_pos_desired[2] = h_t;
      
      g_Controller->SetCartesianPD(Eigen::Vector3d(125,125,125), Eigen::Vector3d(50,50,50));
      g_Controller->SetFootStateDesired(foot_pos_desired, Eigen::Vector3d::Zero());

      // F = ma
      Eigen::Vector3d force_ff = g_Controller->Skeleton()->getMass() * Eigen::Vector3d(0,0,-9.81);
      if(time_now - start_time_ <= 1.0)
      {
      //std::cout << g_Controller->Skeleton()->getMass() << " : " << std::endl;
      force_ff += g_Controller->Skeleton()->getMass() * Eigen::Vector3d(0,0,-a_t);

      }
      g_Controller->SetForceFeedForward(force_ff);

      
  }
  void Enter() 
  { 
    next_state_ = this; 
    // Cache Current Position
    start_pos_ = g_Controller->GetFootPosition();
    // Compute Trajectory from Initial Foot to Stand Height
    double stand_height = .41;
    stand_traj_.Generate(start_pos_[2], -stand_height, 0.0, 0.0, 0.0, 1.0);
    start_time_ = g_world->getTime();
  }
  bool Transition(State *pNextState)
  {
    switch (pNextState->Id())
    {
    case ControllerState::STATE_STAND:
      std::cout << "Transition from Stand to Stand VALID" << std::endl;
      break;
    case ControllerState::STATE_CROUCH:
      std::cout << "Transition from Stand to Crouch VALID" << std::endl;
      next_state_ = pNextState;
      break;
    default:
      std::cout << "Invalid State Transition." << std::endl;
    }
  }
  protected:
    CubicPolynomialTrajectory stand_traj_;
    double start_time_;
    Eigen::Vector3d start_pos_;
};

class IdleState : public State
{
  public:
  IdleState() : State("IDLE", ControllerState::STATE_IDLE)
  {

  }
  void Run()
  {
  }
  void Enter() 
  { 
    std::cout << "In Idle State" << std::endl;
  }
  bool Transition(State *pNextState)
  {
    switch (pNextState->Id())
    {
    // case ControllerState::STATE_CROUCH:
    //   std::cout << "Transition from Stand to Stand VALID" << std::endl;
    //   break;
    case ControllerState::STATE_STAND:
      std::cout << "Transition from Idle to Stand VALID" << std::endl;
      next_state_ = pNextState;
      break;
    default:
      std::cout << "Invalid State Transition." << std::endl;
    }
  }
};

class EmergencyStopState : public State
{
  public:
  EmergencyStopState() : State("EMERGENCY STOP", ControllerState::STATE_ESTOP)
  {

  }
  void Run() {std::cout << "RUNNING ESTOP" << std::endl;}
};


class FiniteStateMachine
{
public:

  FiniteStateMachine(const std::string &name) : name_(name), current_state_(nullptr)
  {
  }

  bool AddState(State *state)
  {
    state->parent_ = this;
    state_list_.emplace(state->Id(), state);
  }

  bool SetDefaultState(State *state)
  {
    default_state_ = current_state_ = state;
  }

  bool SetDefaultState(ControllerState stateId)
  {
    default_state_ = current_state_ = FindState(stateId);
  }

  bool Reset()
  {
    if(current_state_ != nullptr) 
      current_state_->Exit(); // Cleanup old state

    return Start();
  }

  bool Start()
  {
    current_state_ = default_state_;
    current_state_->Enter();
  }

  bool Run()
  {
    if(current_state_ == nullptr)
    {
        std::cout << "[ERROR]: Current state is NULL";
    }

    if(current_state_->InTransition()) // In Transition, run code
    {
      std::cout << "In state transition next state" << std::endl;
      // TODO: Call state transition callbackk
      //current_state_->RunTransition()
    }
    else if(current_state_ != current_state_->NextState()) // Transitioned, cleanup
    {
      // Transitioned
      current_state_->Exit(); // Run State Exit Code
      current_state_ = current_state_->NextState();
      current_state_->Enter(); // Run State Entry Code
      std::cout << "Successfully Transitioned to State: " << current_state_->Name() << std::endl;
    }
    else // Execute Normally
    {
      current_state_->Run();
    }
  }
  void TransitionTo(ControllerState state)
  {
    current_state_->Transition(FindState(state));
  }

  State* FindState(ControllerState eStateId)
  {
    // TODO: Check for valid key
    return state_list_[eStateId];
  }

  // Nomad Data Pointer

protected:
  State *current_state_; // Set Current State
  State *default_state_; // State to reset
  std::string name_;

  std::map<ControllerState, State* > state_list_;
};


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

    step_iter = 0;
  }

  void customPreStep()
  {
    // Use this function to execute custom code before each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
    // leg->getDof(2)->setForce(20.3);

   // std::cout << leg->getJoint("leg_to_world")->getPosition(0) << std::endl;
    if(step_iter < 5)
      return; 

    // Setup
    // Reset Command
   // g_Controller->Reset();

    // Run State Machine
    //g_FSM->Run();

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
    
    step_iter++;
  }

protected:
  SkeletonPtr nomad_;
  SimpleFramePtr hip_target_;
  int step_iter;
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

  virtual bool handle(const ::osgGA::GUIEventAdapter &ea, ::osgGA::GUIActionAdapter &) override
  {
    // Return if null
    if (nullptr == nomad_)
    {
      return false;
    }

    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Up)
      {
        std::cout << "Arrow Key Up!" << std::endl;

        Eigen::Vector3d hipOffset = g_hip_target->getWorldTransform().translation();
        hipOffset = hipOffset + Eigen::Vector3d(0, 0, 0.01);
        g_hip_target->setTranslation(hipOffset);

      //  std::cout << g_hip_target->getWorldTransform().translation() << std::endl;

      //  std::cout << g_hip_target->getTransform(nomad_->getBodyNode("base_link")).translation() << std::endl;
      //  std::cout << "Done " << std::endl;
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Down)
      {
        std::cout << "Arrow Key Down!" << std::endl;

        Eigen::Vector3d hipOffset = g_hip_target->getWorldTransform().translation();
        hipOffset = hipOffset + Eigen::Vector3d(0, 0, -0.01);

        g_hip_target->setTranslation(hipOffset);
        //std::cout << g_hip_target->getWorldTransform().translation() << std::endl;
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_R)
      {
        std::cout << "Reset" << std::endl;
        Reset();
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_S)
      {
          std::cout << "Got Stand Request" << std::endl;
          g_FSM->TransitionTo(ControllerState::STATE_STAND);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_C)
      {
          std::cout << "Got Crouch Request" << std::endl;
          g_FSM->TransitionTo(ControllerState::STATE_CROUCH);
      }
      else if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_E)
      {
        nomad_->getBodyNode("hfe_motor")->addExtForce(Eigen::Vector3d(0,0,-5000), Eigen::Vector3d::Zero(), true, true);
        //std::cout << "FORCE" << std::endl;
      }
    }
  }

  void Reset()
  {
      return;
    // Update Hip Target
    g_hip_target->setTranslation(Eigen::Vector3d(0, -.1, g_stand_height));

    nomad_->getJoint("leg_to_world")->setPositionLimitEnforced(true);
    nomad_->getDof("leg_to_world")->setPosition(0.15);
    nomad_->getJoint("kfe_joint")->setPositionLimitEnforced(true);
    nomad_->getDof("kfe_joint")->setPositionLimits(-1.32, 1.0);

    // Set Positions
    nomad_->getDof("hfe_joint")->setPosition(1.3);
    nomad_->getDof("kfe_joint")->setPosition(-1.3);

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

  //shapeNode->getDynamicsAspect()->setFrictionCoeff(10.0);
  //std::cout << "Mu: " << shapeNode->getDynamicsAspect()->getFrictionCoeff() << std::endl;

  return ground;
}
SkeletonPtr LoadNomad()
{
  dart::utils::DartLoader loader;
  std::string urdf = std::getenv("NOMAD_RESOURCE_PATH");
  urdf.append("/NomadFull/Nomad.urdf");

  SkeletonPtr nomad = loader.parseSkeleton(urdf);

    //nomad->getDof(5)->setName("base_z");
   // std::cout << nomad->getDof(0)->getName() << std::endl;
    nomad->getDof("base_z")->setPosition(0.5);
    nomad->getDof("j_hfe_FL")->setPosition(-M_PI_2);
    nomad->getDof("j_hfe_FR")->setPosition(M_PI_2);
    nomad->getDof("j_hfe_RL")->setPosition(-M_PI_2);
    nomad->getDof("j_hfe_RR")->setPosition(M_PI_2);

    nomad->getJoint("j_kfe_FL")->setPositionLimitEnforced(true);
    nomad->getJoint("j_kfe_FR")->setPositionLimitEnforced(true);
    nomad->getJoint("j_kfe_RL")->setPositionLimitEnforced(true);
    nomad->getJoint("j_kfe_RR")->setPositionLimitEnforced(true);

    nomad->getDof("j_kfe_FL")->setPositionLimits(-2.2, 0.0);
    nomad->getDof("j_kfe_FR")->setPositionLimits(0.0, 2.2);
    nomad->getDof("j_kfe_RL")->setPositionLimits(-2.2, 0.0);
    nomad->getDof("j_kfe_RR")->setPositionLimits(0.0, 2.2);

    
    //nomad->getDof("j_kfe_RL")->setPosition(-2.2);
   // nomad->getDof("j_kfe_FL")->setPosition(-2.2);
   // nomad->getDof("j_kfe_RR")->setPosition(2.2);
   // nomad->getDof("j_kfe_FR")->setPosition(2.2);

  // Set Friction
  //std::cout << nomad->getBodyNode("foot")->getFrictionCoeff() << std::endl;
  //nomad->getBodyNode("foot")->setFrictionCoeff(10.8);


  return nomad;
}

int main(int argc, char *argv[])
{

  // Create dart simulation world
  g_world = World::create();

  // Update Gravity vector to Z down
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  g_world->setGravity(gravity);

  // // Create and add ground to world
  // SkeletonPtr ground = CreateGround();
  // g_world->addSkeleton(ground);

  // // Create and add Nomad test robot to world
  // g_nomad = LoadNomad();
  // g_world->addSkeleton(g_nomad);

//   // Add Controller
//   g_Controller = new LegController(nomad);
//   g_Controller->Reset();

//   g_FSM = new FiniteStateMachine("Control FSM");

//   // TODO: This will be in constructor of subclasses FSM
//   g_FSM->AddState(new CrouchState());
//   g_FSM->AddState(new StandState());
//   g_FSM->AddState(new IdleState());
//   g_FSM->SetDefaultState(ControllerState::STATE_IDLE);
//   g_FSM->Reset();

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