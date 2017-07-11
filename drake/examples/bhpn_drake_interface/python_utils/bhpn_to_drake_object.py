import sys
import pymesh

path_to_objects = '/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/object_conversion_utils/'


def convert(args, transform, mass="25"):
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
        text = text.replace('M', str(mass))
        text = text.replace('R', str(transform[0]))
        text = text.replace('P', str(transform[1]))
        text = text.replace('Y', str(transform[2]))
        text = text.replace('X', str(transform[3]))
        text = text.replace('Y', str(transform[4]))
        text = text.replace('Z', str(transform[5]))
        urdf = open(
            path_to_objects +
            'generated_drake_objects/' +
            name +
            '.urdf',
            'w')
        urdf.write(text)
        urdf.close()
