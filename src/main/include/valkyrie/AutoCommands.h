#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

namespace valor{

class BranchingAutoCommand : public frc2::CommandHelper<frc2::Command, BranchingAutoCommand> {
    public: 
        BranchingAutoCommand(pathplanner::PathPlannerPath p1, pathplanner::PathPlannerPath p2, std::function<bool()> l);
        void Initialize() override;
        bool IsFinished() override;
    
    private:
        pathplanner::PathPlannerPath path1;
        pathplanner::PathPlannerPath path2;
        std::function<bool()> lambda;
};

}