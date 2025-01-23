#include "ReefTracker.h"

ReefTracker::ReefTracker()
{
    init();
}

void ReefTracker::init()
{
    resetReef();
}

void ReefTracker::resetReef()
{
    std::string polePos = "";
    for(int pole = 65; pole < 77; pole++){
        for(int level = 2; level < 5; level++){
            polePos.clear();
            polePos.push_back(pole);
            polePos += std::to_string(level);
            reef[polePos] = false;
        }
    }
}

void ReefTracker::addScore(std::string str)
{
    if(reef.find(str) == reef.end());
    else reef[str] = true;
}

void ReefTracker::removeScore(std::string str)
{
    if(reef.find(str) == reef.end());
    else reef[str] = false;
}

std::vector<std::string> ReefTracker::returnOpenPoles()
{
    std::vector<std::string> openPoles;
    for(auto x : reef){
        if(!x.second) openPoles.push_back(x.first);
    }
    return openPoles;
}

bool ReefTracker::isPoleOpen(std::string str)
{
    if(reef.find(str) == reef.end()) return false;
    return !reef[str];
}

void ReefTracker::printReefState()
{
    for(auto x : reef) std::cout << x.first << " " << x.second << std::endl;
}