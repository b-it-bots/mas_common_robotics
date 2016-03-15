/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * RunScript class, calls system() function 
 * for calling external bash scripts from c++ code
 * 
 */

#include <run_script/run_script.h>
#include <string>
#include <vector>

RunScript::RunScript() : full_path_to_script_(""), script_arguments_(""), script_path_is_set_(false) {}

void RunScript::run(std::string &full_path_to_script)
{
    system(full_path_to_script.c_str());
}

void RunScript::set_script_path(std::string &full_path_to_script)
{
    full_path_to_script_ = full_path_to_script;
    script_path_is_set_ = true;
}

void RunScript::set_script_args(std::vector<std::string> &script_arguments)
{
    args_are_available_ = true;

    for (int i=0; i < script_arguments.size(); i++)
    {
        script_arguments_ += std::string(" ");
        script_arguments_ += script_arguments.at(i);
    }
}

bool RunScript::run()
{
    if (script_path_is_set_)
    {
        if (args_are_available_)
        {
            if (system((full_path_to_script_ + script_arguments_).c_str()) == 0)
            {
                return true;
            }

            return false;
        }
        else
        {
            if (system(full_path_to_script_.c_str()) == 0)
            {
                return true;
            }

            return false;
        }
    }

    return false;
}
