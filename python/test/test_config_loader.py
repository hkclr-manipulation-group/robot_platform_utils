import os
import sys
repo_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
lib_path = [repo_path + "/python/lib/x86", repo_path + "/python/lib/arm64"]
sys.path.extend(lib_path)

import config_loader

if __name__ == "__main__":
    # Example usage
    yaml_node = config_loader.load_yaml("/home/blabla/cuarm_panel_control/cuarm_configuration/dual_v1/config.yaml")
    yaml_node.print()
    yaml_dict = yaml_node.as_dict()

