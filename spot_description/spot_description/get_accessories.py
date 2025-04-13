import os

def get_accessories_from_env():
    spot_accessories: str = os.getenv('SPOT_ACCESSORIES', '')
    spot_accessories_dict: dict = {}
    for accessory in spot_accessories.split(' '):
        if accessory == 'ARM':
            spot_accessories_dict['has_arm'] = 'True'
        elif accessory == 'EAP':
            spot_accessories_dict['has_eap'] = 'True'
        elif accessory == 'EAP2':
            spot_accessories_dict['has_eap_2'] = 'True'
        elif accessory == 'CAM':
            spot_accessories_dict['has_cam_payload'] = 'True'
        elif accessory == 'RL_KIT':
            spot_accessories_dict['has_rl_kit'] = 'True'
        elif accessory == 'REALSENSE':
            spot_accessories_dict['has_realsense'] = 'True'

    return spot_accessories_dict