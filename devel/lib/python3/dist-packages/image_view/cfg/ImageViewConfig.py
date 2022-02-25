## *********************************************************
##
## File autogenerated for the image_view package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'do_dynamic_scaling', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Do dynamic scaling about pixel values or not', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'colormap', 'type': 'int', 'default': -1, 'level': 0, 'description': 'colormap', 'min': -1, 'max': 11, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'NO_COLORMAP', 'type': 'int', 'value': -1, 'srcline': 9, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'NO_COLORMAP', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'AUTUMN', 'type': 'int', 'value': 0, 'srcline': 10, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_AUTUMN', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'BONE', 'type': 'int', 'value': 1, 'srcline': 11, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_BONE', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'JET', 'type': 'int', 'value': 2, 'srcline': 12, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_JET', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'WINTER', 'type': 'int', 'value': 3, 'srcline': 13, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_WINTER', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'RAINBOW', 'type': 'int', 'value': 4, 'srcline': 14, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_RAINBOW', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'OCEAN', 'type': 'int', 'value': 5, 'srcline': 15, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_OCEAN', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'SUMMER', 'type': 'int', 'value': 6, 'srcline': 16, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_SUMMER', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'SPRING', 'type': 'int', 'value': 7, 'srcline': 17, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_SPRING', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'COOL', 'type': 'int', 'value': 8, 'srcline': 18, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_COOL', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'HSV', 'type': 'int', 'value': 9, 'srcline': 19, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_HSV', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PINK', 'type': 'int', 'value': 10, 'srcline': 20, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_PINK', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'HOT', 'type': 'int', 'value': 11, 'srcline': 21, 'srcfile': '/home/joe/kuka_ws/src/image_pipeline/image_pipeline/image_view/cfg/ImageView.cfg', 'description': 'COLORMAP_HOT', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'colormap'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'min_image_value', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Minimum image value for scaling depth/float image.', 'min': 0.0, 'max': inf, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'max_image_value', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Maximum image value for scaling depth/float image.', 'min': 0.0, 'max': inf, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}], 'groups': [], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

ImageView_NO_COLORMAP = -1
ImageView_AUTUMN = 0
ImageView_BONE = 1
ImageView_JET = 2
ImageView_WINTER = 3
ImageView_RAINBOW = 4
ImageView_OCEAN = 5
ImageView_SUMMER = 6
ImageView_SPRING = 7
ImageView_COOL = 8
ImageView_HSV = 9
ImageView_PINK = 10
ImageView_HOT = 11
