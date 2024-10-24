#include "valkyrie/Auto.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
// #include <frc/Filesystem.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
// 
#include <filesystem>

using namespace valor;
using namespace pathplanner;

#define AUTOS_PATH (std::string)"/home/lvuser/deploy/pathplanner/autos/"
#define PATHS_PATH (std::string)"/home/lvuser/deploy/pathplanner/paths/"

Auto::Auto(){
    table = nt::NetworkTableInstance::GetDefault().GetTable("auto");
}

frc2::CommandPtr Auto::makeAuto(std::string autoName){
    return pathplanner::PathPlannerAuto(autoName).ToPtr();
}

frc2::CommandPtr Auto::getSelectedAuto(){
    std::string selection = m_chooser.GetSelected(); 
    return getAuto(selection);
}

frc2::CommandPtr Auto::getAuto(std::string selection) {
    for (uint i = 0; i < loadedAutos.size(); i++) {
        if (loadedAutos[i].first == selection)
            return std::move(loadedAutos[i].second);
    }
    return makeAuto(selection);
}

void Auto::preloadAuto(std::string autoName){
    for (uint i = 0; i < loadedAutos.size(); i++) {
        if (loadedAutos[i].first == autoName) {
            return ;
        }
    }
    loadedAutos.push_back({
        autoName,
        makeAuto(autoName)
    });
}

void Auto::preloadSelectedAuto(){
    if (m_chooser.GetSelected() != "")
        preloadAuto(m_chooser.GetSelected());
}

void Auto::clearAutos(){
    loadedAutos.clear();
}

std::string removeFileType(std::string fileName) {
    return fileName.substr(fileName.find_last_of('/') + 1, fileName.find_last_of('.') - fileName.find_last_of('/') - 1);
}

bool is_alpha(char c){
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}
bool is_caps(char c){
    return (c >= 'A' && c <= 'Z');
}

std::string makeFriendlyName(std::string filename){
    // take last part of the path string when divided with /'s - this should be the filename
    filename = filename.substr(filename.find_last_of('/') + 1);
    std::string n_name = "";
    for (uint i = 0; i < filename.length(); i ++){
        // .'s signify the end of the filename and the start of the file extension
        if (filename[i] == '.'){
            break;
        } else if (filename[i] == '_'){ // replace _'s with spaces for a snake case filename
            // make sure we dont have double spaces
            if (*(n_name.end() - 1) != ' ')
                n_name += ' ';
        } else if (i >= 1 && is_alpha(filename[i]) && is_caps(filename[i]) && !is_caps(filename[i - 1]) && *(n_name.end() - 1) != ' '){ // check for camel case, add space if present
            n_name += ' ';
            n_name += tolower(filename[i]);
        } else if (i == 0){ // first letter should be capitaized
            n_name += toupper(filename[i]);
        } else{
            n_name += tolower(filename[i]);
        }
    }
    return n_name;
}

std::vector<std::string> listDirectory(std::string path_name){
    std::vector<std::string> files;

    for (const auto & entry : std::filesystem::directory_iterator(path_name)){
        std::string path = entry.path();
        if (path.find(".path") != std::string::npos || path.find(".auto") != std::string::npos) {
            files.push_back(entry.path());
        }
    }
    return files;
}

void Auto::fillAutoList(){
    for (std::string path : listDirectory(AUTOS_PATH)){
        m_chooser.AddOption(makeFriendlyName(removeFileType(path)), removeFileType(path));
    }
    frc::SmartDashboard::PutData(&m_chooser);
}
