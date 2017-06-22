#!/usr/bin/python
import sys
import pymesh

path_to_objects = '/home/tristanthrush/FakeDesktop/spartan/drake/drake/examples/bhpn_drake_interface/object_conversion_utils/'

def main():
    # print command line arguments
    for arg in sys.argv[1:]:
	mesh = pymesh.load_mesh(arg)
	name = arg.split('.')[0]
	name = arg.split('/')[-1]
	pymesh.save_mesh(path_to_objects + 'generated_drake_objects/' + name + '.obj', mesh)
	template_urdf = open(path_to_objects + 'template.urdf', 'r')
	text = template_urdf.read()
	template_urdf.close()
	text = text.replace('object_name', name)
	urdf = open(path_to_objects + 'generated_drake_objects/' + name + '.urdf', 'w')
	urdf.write(text)
	urdf.close()
	
	
     
if __name__ == "__main__":
    main()
