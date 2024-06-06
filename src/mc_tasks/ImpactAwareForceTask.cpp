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


  type_ = "impact_aware_force";
  name_ = "impact_aware_force_" + robots.robot(rIndex_).name();
  reset();
}

void ImpactAwareForceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryBase::load(solver, config);

}

void ImpactAwareForceTask::update(mc_solver::QPSolver & s)
{
    const mc_rbdyn::Robot & robot = frame_->robot();


}

void ImpactAwareForceTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);

}

void ImpactAwareForceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
                                                                  

}

void ImpactAwareForceTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::removeFromGUI(gui);
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