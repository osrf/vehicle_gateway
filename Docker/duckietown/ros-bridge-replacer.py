""" This script reads the `bridge.yml` file and replaces the value of the `topic` key in the first two elements of the list with a string that uses environment variables. The string is of the format: `$(env veh)/$(env ROBOT_HARDWARE)_camera_node` where the `env` keys are taken from the shell environment. """

import yaml
import os
import jinja2

# Read the environment variables
file_name = 'bridge.yml'

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
            
substitute_env_variables(file_name)