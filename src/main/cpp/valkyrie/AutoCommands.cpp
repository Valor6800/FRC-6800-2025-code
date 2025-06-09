#include "valkyrie/AutoCommands.h"

using namespace valor;

BranchingAutoCommand::BranchingAutoCommand(pathplanner::PathPlannerPath p1, pathplanner::PathPlannerPath p2, std::function<bool()> l) : path1(p1), path2(p2), lambda(l)
{
}

void BranchingAutoCommand::Initialize(){
    if(lambda()){
        auto command = pathplanner::AutoBuilder::followPath(std::make_shared<pathplanner::PathPlannerPath>(path1));
        command.Schedule();
    }
    else{
        auto command = pathplanner::AutoBuilder::followPath(std::make_shared<pathplanner::PathPlannerPath>(path1));
        command.Schedule();
    }
}

bool BranchingAutoCommand::IsFinished(){
    return true;
}