""" This script reads the `gazebo-bridge-config.yml` file and replaces the value of the `veh`
key with the `VEH` environment variables. The string is of the format: 
`$(env veh)/virtual_camera_node` where the `env` keys are taken from the shell environment. """

import yaml
import os
import jinja2

# Read the environment variables
file_name = "gazebo-bridge-config.yaml"

def substitute_env_variables(file_name):
    env = jinja2.Environment(loader=jinja2.FileSystemLoader('/data/config'))
    template = env.get_template(file_name + '.j2')
    # Read the environment variables
    substitution_variables = {'veh': os.getenv('VEH')}
    
    env = jinja2.Environment(loader=jinja2.FileSystemLoader('/data/config'))
    template = env.get_template(file_name + '.j2')
    data = template.render(substitution_variables)
    
    with open(f"/data/config/{file_name}", 'w') as stream:
        try:
            # Parse the data as YAML
            yaml.dump(yaml.safe_load(data), stream)
        except yaml.YAMLError as exc:
            print(exc)

substitute_env_variables("gazebo-bridge-config.yaml")
