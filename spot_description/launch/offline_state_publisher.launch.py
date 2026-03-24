import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from spot_description.get_accessories import get_accessories_from_env

joint_names = [
    "front_left_hip_x",
    "front_left_hip_y",
    "front_left_knee",
    "front_right_hip_x",
    "front_right_hip_y",
    "front_right_knee",
    "rear_left_hip_x",
    "rear_left_hip_y",
    "rear_left_knee",
    "rear_right_hip_x",
    "rear_right_hip_y",
    "rear_right_knee",
    "arm0_shoulder_yaw",
    "arm0_shoulder_pitch",
    "arm0_elbow_pitch",
    "arm0_elbow_roll",
    "arm0_wrist_pitch",
    "arm0_wrist_roll",
    "arm0_fingers",
    "body_height_joint",
    "body_yaw_joint",
    "body_pitch_joint",
    "body_roll_joint",
    "body_x",
    "body_y",
    "body_or",
]

defaults = {
    'docked': [
        -0.3428281247615814,
        1.0187467336654663,
        -2.605299711227417,
        0.29793134331703186,
        1.0114928483963013,
        -2.5962142944335938,
        -0.22116026282310486,
        1.0229789018630981,
        -2.6222493648529053,
        0.18044471740722656,
        1.0009557008743286,
        -2.6095738410949707,
        2.8371810913085938e-05,
        -3.1344666481018066,
        3.1335628032684326,
        1.5701513290405273,
        0.0001703500747680664,
        -1.5690388679504395,
        -0.011029481887817383,
        0.010768269089347183,
        0.0,
        0.00928758434112388,
        0.0010131850667112088,
        0.0,
        0.0,
        0.0,
    ],
    'standing': [
        -0.0058014001697301865,
        0.7929132580757141,
        -1.5593334436416626,
        -0.007982775568962097,
        0.7943171262741089,
        -1.5458697080612183,
        0.0062673864886164665,
        0.7941105961799622,
        -1.5512280464172363,
        0.012335322797298431,
        0.8197641372680664,
        -1.55510675907135,
        -0.0002739429473876953,
        -3.11625075340271,
        3.13369083404541,
        1.5699230432510376,
        0.0003256797790527344,
        -1.5691757202148438,
        -0.010957598686218262,
        0.5210924836499518,
        0.0,
        -0.000829264610125228,
        -0.0015327716047118937,
        0.0,
        0.0,
        0.0,

    ],
    'ready': [
        -0.00678610522300005,
        0.6703529953956604,
        -1.518938660621643,
        -0.009314254857599735,
        0.671734094619751,
        -1.5021194219589233,
        0.004862139467149973,
        0.6696428656578064,
        -1.5052406787872314,
        0.012546989135444164,
        0.6965122818946838,
        -1.5123471021652222,
        -0.000362396240234375,
        -0.899552583694458,
        1.8030341863632202,
        0.007572650909423828,
        -0.8988518714904785,
        -0.0024976730346679688,
        -0.011017560958862305,
        0.5265096429717088,
        0.0,
        0.0023632052482177295,
        -0.0033911078453983754,
        0.0,
        0.0,
        0.0,
    ],
    'unstowed': [
        -0.005910136271268129,
        0.7761390209197998,
        -1.549218773841858,
        -0.008832432329654694,
        0.7776713371276855,
        -1.5329267978668213,
        0.006094180513173342,
        0.7778038382530212,
        -1.5406819581985474,
        0.014208528213202953,
        0.8038771748542786,
        -1.5456631183624268,
        -0.029943227767944336,
        -2.634772539138794,
        2.914163589477539,
        0.024838924407958984,
        -0.24115753173828125,
        -0.020923614501953125,
        -0.010993599891662598,
        0.5232863240570801,
        0.0,
        0.00021981033690376085,
        -0.0031286977513961018,
        0.0,
        0.0,
        0.0,
    ],
    'namaste': [
        -0.005910136271268129,
        0.7761390209197998,
        -1.549218773841858,
        -0.008832432329654694,
        0.7776713371276855,
        -1.5329267978668213,
        0.006094180513173342,
        0.7778038382530212,
        -1.5406819581985474,
        0.014208528213202953,
        0.8038771748542786,
        -1.5456631183624268,
        -0.00415325,
        -1.3191,
        2.291,
        0.067544,
        -1.8325,
        -0.0605698,
        -0.010993599891662598,
        0.5232863240570801,
        0.0,
        0.00021981033690376085,
        -0.0031286977513961018,
        0.0,
        0.0,
        0.0,
    ],
    'carry': [
        -0.005910136271268129,
        0.7761390209197998,
        -1.549218773841858,
        -0.008832432329654694,
        0.7776713371276855,
        -1.5329267978668213,
        0.006094180513173342,
        0.7778038382530212,
        -1.5406819581985474,
        0.014208528213202953,
        0.8038771748542786,
        -1.5456631183624268,
        -0.0041,
        -1.91044,
        2.50288,
        0.067544,
        -0.544375,
        -0.0605698,
        -0.010993599891662598,
        0.5232863240570801,
        0.0,
        0.00021981033690376085,
        -0.0031286977513961018,
        0.0,
        0.0,
        0.0,
    ],
    'namaste_carry': [
        -0.005910136271268129,
        0.7761390209197998,
        -1.549218773841858,
        -0.008832432329654694,
        0.7776713371276855,
        -1.5329267978668213,
        0.006094180513173342,
        0.7778038382530212,
        -1.5406819581985474,
        0.014208528213202953,
        0.8038771748542786,
        -1.5456631183624268,
        -0.0041,
        -1.91044,
        2.50288,
        0.067544,
        -1.8325,
        -0.0605698,
        -0.010993599891662598,
        0.5232863240570801,
        0.0,
        0.00021981033690376085,
        -0.0031286977513961018,
        0.0,
        0.0,
        0.0,
    ],
}

def launch_joint_states(context, *args, **kwargs) -> dict[str: str]:
    name_param: LaunchConfiguration = kwargs['name']
    urdf_param: ParameterValue = kwargs['urdf']

    name: str = name_param.perform(context)
    urdf: str = urdf_param.evaluate(context)
    
    default_joint_vals = [{f'zeros.{joint_name}':val} for joint_name, val in zip(joint_names, defaults[name])]

    fake_joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'robot_description':urdf},
            *default_joint_vals,
            {'source_list':['/spot_extra_joint_states']}
        ],
        remappings=[
            ('/joint_states', '/spot_driver/joint_states')
        ]
    ),

    return fake_joint_state_pub

def generate_launch_description():

    spot_accessory_dict = get_accessories_from_env()

    launch_args = [
        DeclareLaunchArgument('has_arm',
            description='Boolean. Include the Spot Arm.',
            default_value=spot_accessory_dict.get('has_arm', 'False')),
            
        DeclareLaunchArgument('has_eap',
            description='Boolean. Include the Enhanced Autonomy package (EAP)',
            default_value=spot_accessory_dict.get('has_eap', 'False')),

        DeclareLaunchArgument('has_eap_2',
            description='Boolean. Include the Updated Enhanced Autonomy package (EAP2)',
            default_value=spot_accessory_dict.get('has_eap_2', 'False')),

        DeclareLaunchArgument('has_rl_kit',
            description='Boolean. Include the RL Research Kit mounting set',
            default_value=spot_accessory_dict.get('has_rl_kit', 'False')),

        DeclareLaunchArgument('has_realsense',
            description='Boolean. Include an arm mounted Realsense D435',
            default_value=spot_accessory_dict.get('has_realsense', 'False')),

        DeclareLaunchArgument('has_cam_payload',
            description='Boolean. Include the CAM payload',
            default_value=spot_accessory_dict.get('has_cam_payload', 'False')),

        DeclareLaunchArgument('kinematic_model',
            description='The kinematic model to use for the Spot description',
            choices=['none', 'body_assist', 'mobile_manipulation'],
            default_value='none'),

        DeclareLaunchArgument('use_proprietary_meshes',
            description='Whether to use proprietary meshes',
            default_value='False'),

        DeclareLaunchArgument(
            'proprietary_pkg',
            description='Name of the package containing proprietary meshes',
            default_value='spot_proprietary_description'),
        
        DeclareLaunchArgument(
            'proprietary_mesh_format',
            description='File extension format for proprietary mesh files',
            default_value='dae',
            choices=['dae', 'stl', 'obj']),

        DeclareLaunchArgument('configuration',
            description='The configuration to emulate for the Spot robot',
            choices=['docked', 'standing', 'ready', 'unstowed', 'namaste', 'carry', 'namaste_carry'],
            default_value='standing'
        )
    ]

    launch_arg_names = ['has_arm', 'has_eap', 'has_eap_2', 'has_rl_kit', 'has_realsense', 'has_cam_payload', 'kinematic_model', 'use_proprietary_meshes', 'proprietary_pkg', 'proprietary_mesh_format']
    xacro_command_args = [elem for arg_name in launch_arg_names for elem in (f' {arg_name}:=', LaunchConfiguration(arg_name))]
    
    # Build the URDF from the xacro, applying specified hardware accessories.
    xacro_path = PathJoinSubstitution([FindPackageShare('spot_description'), 'urdf', 'spot.urdf.xacro'])
    urdf_param = ParameterValue(Command(['xacro ', xacro_path, *xacro_command_args]), value_type=str)

    # Camera transforms
    camera_transforms = IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution([FindPackageShare('spot_description'), 'launch', 'fake_camera_transforms.launch.py']),
        launch_arguments=[('has_arm', LaunchConfiguration('has_arm'))]
    )

    # Base footprint
    base_footprint_pub = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'body', '--z', '0.52']
    )

    return launch.LaunchDescription([
        *launch_args,
        camera_transforms,
        base_footprint_pub,

        OpaqueFunction(function=launch_joint_states, kwargs={'name': LaunchConfiguration('configuration'), 'urdf':urdf_param}),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': urdf_param}],
            remappings=[
                ('/joint_states', '/spot_driver/joint_states')
            ]
        )
    ])
