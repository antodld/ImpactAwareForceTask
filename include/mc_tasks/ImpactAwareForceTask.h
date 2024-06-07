#pragma once

// includes
//
#include <array>

#include <tasks/config.hh>

// Eigen
#include <eigen3/Eigen/Core>


#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <mc_tasks/AdmittanceTask.h>

namespace mc_tasks
{
namespace force
{
struct MC_TASKS_DLLAPI ImpactAwareForceTask : public AdmittanceTask
{

public:


    ImpactAwareForceTask(const mc_rbdyn::RobotFrame & frame, double weight ,double mu = 5.0, double n = 1.0, double m = 1);


    void reset() override
    {
        AdmittanceTask::reset();
    }

    /*! \brief Load parameters from a Configuration object */
    void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

 



    void update(mc_solver::QPSolver & s) override;




protected:

    void addToGUI(mc_rtc::gui::StateBuilder &) override;

    void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

    void addToLogger(mc_rtc::Logger & logger) override;


private:


    size_t rIndex_;

    /** True if added to solver */
    bool inSolver_ = false;
    /** Robot handled by the task */

    double weight_ = 0;

    double mu_ = 1;
    double m_ = 1;
    Eigen::VectorXd velGain_;
    double n_ = 1.5;


};

using ImpactAwareForceTaskPtr = std::shared_ptr<ImpactAwareForceTask>;

} //namespace force

} //namespace tasks