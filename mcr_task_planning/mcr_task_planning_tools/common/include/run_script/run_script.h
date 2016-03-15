/*  
* Copyright [2016] <Bonn-Rhein-Sieg University>  
*  
* Author: Oscar Lima (olima_84@yahoo.com)
* 
* This common library calls an external script
* 
* 1. Receives the full path where the script is located
* 2. Runs it
*   
*/

#ifndef RUN_SCRIPT_RUN_SCRIPT_H
#define RUN_SCRIPT_RUN_SCRIPT_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

class RunScript
{
    public:
        // constructor
        RunScript();

        // calls an external script
        void run(std::string &full_path_to_script);

        // to set the script path and store in member variable
        void set_script_path(std::string &full_path_to_script);

        // to set script arguments
        void set_script_args(std::vector<std::string> &script_arguments);

        // call an external script previously set by this class
        // return true if path to script is set, false if not
        bool run();

    private:
        // string to store the script path
        std::string full_path_to_script_;

        // string to store the script arguments
        std::string script_arguments_;

        // flag to indicate if the script path is set
        bool script_path_is_set_;

        // flag to indicate that arguments are available for running the script
        bool args_are_available_;
};
#endif  // RUN_SCRIPT_RUN_SCRIPT_H
