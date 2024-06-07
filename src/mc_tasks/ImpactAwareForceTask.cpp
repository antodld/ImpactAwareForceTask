// associated header
#include "../../include/mc_tasks/ImpactAwareForceTask.h"
// includes
// std
#include <cmath>
#include <iterator>
#include <set>

// Eigen
#include <Eigen/Geometry>

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

#include <mc_tasks/MetaTaskLoader.h>  

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>




namespace mc_tasks
{
namespace force
{

ImpactAwareForceTask::ImpactAwareForceTask(const mc_rbdyn::RobotFrame & frame, double weight ,double mu , double n , double m)
: AdmittanceTask(frame,0,weight)
{

  rIndex_ = frame.robot().robotIndex();
  type_ = "impact_aware_force";
  name_ = "impact_aware_force_" + robots.robot(rIndex_).name();
  reset();
}

void ImpactAwareForceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  AdmittanceTask::load(solver, config);

  mu_ = config("mu",1);
  n_ = config("n",2);
  m_ = config("m",1);
  weight(config("weight",1000));
  refVel_ = Eigen::Vector6d::Zero();
 

}

void ImpactAwareForceTask::update(mc_solver::QPSolver & s)
{
    const mc_rbdyn::Robot & robot = frame_->robot();

    const Eigen::VectorXd velError = (sva::PTransformd(frame_->position().rotation()) * frame_->velocity()).vector() - refVel_;

    if(frame_->hasForceSensor())
    {
      measuredWrench_ = frame_->wrench();
    }

    for(size_t i = 0 ; i < damping_.rows() ; i++)
    {
      damping_(i) = (mu_ /m_)* std::pow(std::abs(velError(i)),n_-1);
    }
    // damping( (mu_ /m_)* std::pow(velError.norm(),n_-1));
    refAccel(sva::MotionVecd((measuredWrench_).vector())/m_);


}

void ImpactAwareForceTask::addToLogger(mc_rtc::Logger & logger)
{
  AdmittanceTask::addToLogger(logger);

}

void ImpactAwareForceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  AdmittanceTask::addToGUI(gui);
  gui.addElement({"Tasks",name_,"Gains"}, 
    mc_rtc::gui::NumberInput("mu",[this]()-> const double {return mu_;},[this]( double s){mu_ =s;}),
    mc_rtc::gui::NumberInput("m",[this]()-> const double {return m_;},[this]( double s){m_ =s;}),
    mc_rtc::gui::NumberInput("n",[this]()-> const double {return n_;},[this]( double s){n_ =s;})
    );                                                                  

}

void ImpactAwareForceTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  AdmittanceTask::removeFromGUI(gui);
  gui.removeCategory({"Task",name_});

}


} //namespace force

} //namespace tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "impact_aware_force",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {

      auto frame = [&]() -> std::string
      {
        return config("frame");
      }();
      auto rIndex = robotIndexFromConfig(config, solver.robots(), "impact_aware_force");

      auto t = std::make_shared<mc_tasks::force::ImpactAwareForceTask>(solver.robots().robot(rIndex).frame(frame),10);
      t->reset();
      t->load(solver, config);
      return t;
    });
}