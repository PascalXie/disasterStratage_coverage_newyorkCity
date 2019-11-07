import bpy
import random

def main():
	#
	# clean
	#
	override = bpy.context.copy()
	list_objects_deleted = []
	
	# print all objects
	for obj in bpy.data.objects:
		print(obj.name)
		if 'group' not in obj.name:
			print('obj deleted')
			list_objects_deleted.append(obj)
	
	override['selected_objects'] = list(list_objects_deleted)
	bpy.ops.object.delete(override)
	print("Step 0 : \"Cleaning objects\" done")
	
	
	#
	# get coordinates
	#
	# https://docs.blender.org/api/current/bpy.types.Mesh.html?highlight=vertices#bpy.types.Mesh.vertices
	f = open('/Users/pascal/动画游戏科技-2019/10月-科技-WSN灾难救援部署问题/地理模型-Blender/python脚本/data_buildings.txt','w')

	for m in bpy.data.meshes:
		#print(m.name)
		print('length {}'.format(len(m.vertices)))

		# get rid of the faces or lines, keep the buildings
		if len(m.vertices)<6:
			#print('!!! {}'.format(m.name))
			continue

		if 'ID' not in m.name:
			continue

		# compute center of the building
		center = [0.,0.,0.]
		for v in m.vertices:
			#print('v  {} '.format(v.co))
			center[0] += v.co[0]
			center[1] += v.co[1]
			center[2] += v.co[2]

		#print('center total {} '.format(center))

		for index in range(3):
			center[index] /= float(len(m.vertices))

		#print('center {} '.format(center))

		# compute the radius for each building
		radius = 0.
		for v in m.vertices:
			r = 0.
			r = r + (v.co[0]-center[0])**2
			r = r + (v.co[1]-center[1])**2
			r = r**0.5
			if r>radius:
				radius = r

		print('radius {} '.format(radius))

		# write
		line = m.name
		line = line + " " + str(center[0]) + " " + str(center[1])
		line = line + " " + str(radius) 
		line = line + "\n"
		f.write(line)


	f.close()

	return


print('hello')
main()
