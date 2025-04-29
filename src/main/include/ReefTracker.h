#pragma once
#include <iostream>
#include <unordered_map>
#include <string>
#include <vector>

class ReefTracker
{
    public:
        ReefTracker();
        void init();
        void addScore(std::string);
        void removeScore(std::string);
        std::vector<std::string> returnOpenPoles();
        bool isPoleOpen(std::string);
        void printReefState();
        void resetReef();
    private:
        std::unordered_map<std::string, bool> reef;
};