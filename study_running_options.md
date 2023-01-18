# Log of params/args


## Arg types

* S = study-specific args: same arg used for all trials in study (set these up as constants at the top of your study running script)
* T = trial-specific args: args that are specific to certain trials in a study (set these up in a config file)
* R = run-specific args: args that may change for the same trial and/or study (more like run options such as enable vis or running in sim)



* sim - R
* study_version_name: pilot_study_v1 - S
* input_interface: generic_joystick - S
* num_input_dofs: 2 - S
* control_hz: 50.0 - S
* max_linear_velocity: None - S
* max_angular_velocity: None - S
* goal_objects: [kinova_holder] - T
* record_data: False - R
* log_dir: None - T
* enable_vis: True - R
* init_joint_config - T
* env files - T
* env model dir - S

