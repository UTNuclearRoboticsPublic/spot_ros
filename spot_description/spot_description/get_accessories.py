import os
import ament_index_python

# All currently supported payloads for Spot
accessory_to_arg_map = {
    'ARM'   : 'has_arm',
    'EAP'   : 'has_eap',
    'EAP2'  : 'has_eap_2',
    'RL_KIT': 'has_rl_kit', 
}

def get_accessories_from_env():
    spot_accessories: str = os.getenv('SPOT_ACCESSORIES', '')
    spot_accessories_dict: dict = {}
    for accessory in spot_accessories.split(' '):
        if accessory in accessory_to_arg_map:
            arg_name = accessory_to_arg_map[accessory]
            spot_accessories_dict[arg_name] = 'True'

    try:
        ament_index_python.get_package_prefix('spot_proprietary_description')
        spot_accessories_dict['use_proprietary_meshes'] = 'True'
    except ament_index_python.PackageNotFoundError:
        pass

    return spot_accessories_dict