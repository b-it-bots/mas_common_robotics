#! /bin/bash

# --------------CHECK ARGS--------

# check if number of received arguments is ok, if not then prints USAGE instructions
if [[ $# != 3 ]]; then
    printf 'USAGE : domain_path problem_number mercury_path\n\n'
    echo '1 argument = path to domain'
    echo '2 argument = path to pddl problem definition'
    echo '3 argument = path to mercury_planner binaries    (path to seq-sat-mercury code)'
    printf '\nEXAMPLE :\n'
    echo './mercury'
    echo '/home/user/indigo/src/mas_common_robotics/mcr_task_planning/mcr_knowledge/common/pddl/example_domain/domain.pddl'
    echo '/home/user/indigo/src/mas_common_robotics/mcr_task_planning/mcr_knowledge/common/pddl/example_domain/problems/p1.pddl'
    echo '/home/user/indigo/src/mas_third_party_software/mercury_planner/build/Mercury-fixed/seq-sat-mercury'
    printf '\nWARNING : Script execution will be aborted\n'
    exit 1
fi


#--------------SETUP--------------

# Paths to planner components

# contains the PDDL domain definition
PDDL_DOMAIN_PATH=${1}

# contains PDDL problem instance
PDDL_PROBLEM_PATH=${2}

# the directoy which holds mercury_planner code
BASEDIR=${3}'/src'

# full path to translate.py file
TRANSLATE_PATH="$BASEDIR/translate/translate.py"

# full path to preprocess binary
PREPROCESS_PATH="$BASEDIR/preprocess/preprocess"

# full path to search binary
SEARCH_PATH="$BASEDIR/search/downward-1"

# command used to determine how much time took to plan
TIME="command time --output=elapsed.time --format=%S\n%U\n"

# command to run the python file
PYTHON_COMMAND="python"


#--------------RUN----------------
echo "1. Running translator"
$TIME $PYTHON_COMMAND $TRANSLATE_PATH $PDDL_DOMAIN_PATH $PDDL_PROBLEM_PATH

echo "2. Running preprocessor"
$TIME --append "$PREPROCESS_PATH" < output.sas

echo "3. Running search"
$TIME --append "$SEARCH_PATH" \
--heuristic "hrb=RB(cost_type=1, extract_plan=true, next_red_action_test=true, applicable_paths_first=true, use_connected=true)" \
--heuristic "hlm2=lmcount(lm_rhw(reasonable_orders=true,lm_cost_type=2,cost_type=2))" \
--search "lazy_wastar([hrb,hlm2],preferred=[hrb,hlm2],w=3)" \
--plan-file mercury.plan < output

#--------------CLEAN--------------
#remove output file if exists
file=output
if [ ! -e "$file" ]; then
    echo "nothing to clean for : output"
else 
    echo "clean : removing file output"
    rm output
fi

#remove output file if exists
file=output.sas
if [ ! -e "$file" ]; then
    echo "nothing to clean for : output.sas"
else 
    echo "clean : removing file output.sas"
    rm output.sas
fi

#remove output file if exists
file=plan_numbers_and_cost
if [ ! -e "$file" ]; then
    echo "nothing to clean for : plan_numbers_and_cost)"
else 
    echo "clean : removing file plan_numbers_and_cost"
    rm plan_numbers_and_cost
fi
