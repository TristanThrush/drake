import sys
import pymesh

path_to_objects = '/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/object_conversion_utils/'


def convert(args):
    args = args.split(' ')
    for arg in args:
        mesh = pymesh.load_mesh(arg)
        name = arg.split('/')[-1]
        name = name.split('.')[0]
        print 'name: ', name
        pymesh.save_mesh(
            path_to_objects +
            'generated_drake_objects/' +
            name +
            '.obj',
            mesh)
        template_urdf = open(path_to_objects + 'template.urdf', 'r')
        text = template_urdf.read()
        template_urdf.close()
        text = text.replace('object_name', name)
        urdf = open(
            path_to_objects +
            'generated_drake_objects/' +
            name +
            '.urdf',
            'w')
        urdf.write(text)
        urdf.close()
