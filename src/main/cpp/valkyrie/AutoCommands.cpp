#include "valkyrie/AutoCommands.h"

using namespace valor;

BranchingAutoCommand::BranchingAutoCommand(pathplanner::PathPlannerPath p1, pathplanner::PathPlannerPath p2, std::function<bool()> l) : path1(p1), path2(p2), lambda(l), command(frc2::cmd::None())
{
}

void BranchingAutoCommand::Initialize(){
    if(lambda()){
        command = pathplanner::AutoBuilder::followPath(std::make_shared<pathplanner::PathPlannerPath>(path1));
        command.Schedule();
    }
    else{
        command = pathplanner::AutoBuilder::followPath(std::make_shared<pathplanner::PathPlannerPath>(path1));
        command.Schedule();
    }
}

bool BranchingAutoCommand::IsFinished(){
    return command.get()->IsFinished();
}