# mcr_default_env_config

## Introduction

This package organizes map information parameters to upload to script_server.

## Launch Files

1. upload_params.launch: Upload parameters.  

### Parameters
* robot_env: ROBOT_ENV environment parameter which provides the name of the environment which match with a folder name in the current repository.
* atHome : If false, upload just navigation_goals file poses. If true add speech_names, speech_places, speech_actions, speech_objects yaml poses file in addition to the navigation_goals.

