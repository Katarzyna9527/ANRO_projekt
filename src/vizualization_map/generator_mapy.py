#!/usr/bin/python
# -*- coding: utf-8 -*- 

print "Kamila's Map Generator!\n"
print "+ kierownik w małym stopniu\n"

class crossway:

	def __init__(self,ID,neighbors,lenghts):
		self.ID = ID
		self.neighbors = neighbors
		self.lenghts = lenghts

	x=0
	y=0
	placed=False

def set_cross(cross):
	for j in range(4):
		neighborID = int(cross.neighbors[j])
		if neighborID!=0:
			neighbor = crossway_list[neighborID-1]
			if neighbor.placed==False:
				lenght = int(cross.lenghts[j])*5
				if j==0:
					neighbor.y = cross.y + lenght
					neighbor.x = cross.x
				if j==1:
					neighbor.x = cross.x + lenght
					neighbor.y = cross.y
				if j==2:
					neighbor.y = cross.y - lenght
					neighbor.x = cross.x
				if j==3:
					neighbor.x = cross.x - lenght
					neighbor.y = cross.y
				neighbor.placed = True

used_connection = []
def add_to_used_connection(a, b):
	a=int(a)
	b=int(b)
	if a<b:
		tmp=str(a)+"-"+str(b)
	else:
		tmp=str(b)+"-"+str(a)
		
	if tmp in used_connection:
		return False
	else: 
		used_connection.append(tmp)
		return True


	
crossway_list = []

ID=0
neighbors = [0,0,0,0]
lenghts = [0,0,0,0]

line_type = 0

conffile = open('./src/map/src/conffile', 'r')

for line in conffile:
   line_type=line_type+1
   if len(line.split("\n")[0])==0:
	break
   if line_type==1: 
	ID = int(line.split("\n")[0])
   if line_type==2:
	contener = line.split("\n")[0].split(" ")
	neighbors = [0,0,0,0]
	for j in range(1,5):
	   neighbors[j-1] = contener[j]
   if line_type==3:
	contener = line.split("\n")[0].split(" ")
	lenghts = [0,0,0,0]
	for j in range(1,5):
	   lenghts[j-1] = contener[j]
	line_type=0
	crossway_list.append(crossway(ID, neighbors, lenghts))

conffile.close()

for cross in crossway_list:
	set_cross(cross)
	print "Crossing "+str(cross.ID)+": "+str(cross.x)+" "+str(cross.y)

fileout = open('./src/vizualization_map/mapa.urdf', 'w')

cross_link=open('./src/vizualization_map/templates/cross_link', 'r').read()
cross_join=open('./src/vizualization_map/templates/cross_join', 'r').read()
road_link=open('./src/vizualization_map/templates/road_link', 'r').read()
road_join=open('./src/vizualization_map/templates/road_join', 'r').read()
fileout.write("<?xml version=\"1.0\"?><robot name=\"lights\"><link name=\"base_link\"></link>")
for cross in crossway_list:
	
	line = cross_link.replace("{id}",str(cross.ID))
	fileout.write(line)
	
	line = cross_join.replace("{id}",str(cross.ID))
	line = line.replace("{x}",str(cross.x))
	line = line.replace("{y}",str(cross.y))
	fileout.write(line)

	for i in range(0, len(cross.neighbors)):
		if cross.neighbors[i]!="0":
			if add_to_used_connection(cross.ID, cross.neighbors[i]):
				tmp=crossway_list[int(cross.neighbors[i])-1]
				print "Connection between "+str(cross.ID)+" and "+str(cross.neighbors[i])+" was added"
				line = road_link.replace("{s_id}",str(cross.ID))
				line = line.replace("{d_id}",str(cross.neighbors[i]))	
				if i==1:
					width = tmp.x-cross.x
					height = 4
					print "Crossing "+str(cross.ID)+" and "+str(cross.neighbors[i])+": "+str(width)+" "+str(height)
				elif i==2:
					width = 4
					height = cross.y-tmp.y
					print "Crossing "+str(cross.ID)+" and "+str(cross.neighbors[i])+": "+str(width)+" "+str(height)
					
				line = line.replace("{height}",str(height))
				line = line.replace("{width}",str(width))
				fileout.write(line)
				
				line = road_join.replace("{s_id}",str(cross.ID))
				line = line.replace("{d_id}",str(cross.neighbors[i]))
				if i==1:
					line = line.replace("{x_offset}",str((tmp.x-cross.x)/2))
				elif i==2:
					line = line.replace("{y_offset}",str((tmp.y-cross.y)/2))
					
				line = line.replace("{x_offset}",str(0))
				line = line.replace("{y_offset}",str(0))

				fileout.write(line)
				
fileout.write("</robot>")		
fileout.close()




