#!/usr/bin/python

print "Kamila's Map Generator!"

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
				'''print neighborID, neighbor.x, neighbor.y'''
				lenght = int(cross.lenghts[j])*5
				if j==0:
					neighbor.x = cross.x - lenght -5
					neighbor.y = cross.y
				if j==1:
					neighbor.y = cross.y + lenght +5
					neighbor.x = cross.x
				if j==2:
					neighbor.x = cross.x + lenght +5
					neighbor.y = cross.y
				if j==3:
					neighbor.y = cross.y - lenght -5
					neighbor.x = cross.x
				neighbor.placed = True
				'''print neighborID, neighbor.x, neighbor.y'''
				set_cross(neighbor)


crossway_list = []

ID=0
neighbors = [0,0,0,0]
lenghts = [0,0,0,0]

line_type = 0

conffile = open('../map/src/conffile', 'r')

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

"""for i in range(0,len(crossway_list)):
	print crossway_list[i].ID
	print crossway_list[i].neighbors
	print crossway_list[i].lenghts"""

crossway_list[0].placed=True
set_cross(crossway_list[0])


	
filein = open('mapa_blank.urdf', 'r')
fileout = open('mapa.urdf', 'w')

for line in filein:
	fileout.write(line)
	if '<!-- links  -->' in line:
		break

for i in range(0,len(crossway_list)):
	
	filedroga = open('templates/link_droga', 'r')
	for line in filedroga:
		line = line.replace("{id}",str(crossway_list[i].ID))
		fileout.write(line)
	filedroga.close()
	connections=0
	for j in range(0,len(crossway_list[i].neighbors)):
		if crossway_list[i].neighbors[j]!="0":
			connections=connections+1

	if crossway_list[i].neighbors[0]!="0":
		filedrogadir = open('templates/link_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "N")
			line = line.replace("{width}","0.5").replace("{height}", "4")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/link_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "N")
				fileout.write(line)
			filelatarnia.close()

	if crossway_list[i].neighbors[1]!="0":

		numberOfRoadKafeleks = int(crossway_list[i].lenghts[1])
		if numberOfRoadKafeleks!=0:
			for j in range(0,numberOfRoadKafeleks):
				'''print numberOfRoadKafeleks, crossway_list[i].ID'''
				filedroga = open('templates/link_droga', 'r')
				for line in filedroga:
					line = line.replace("{id}",str(crossway_list[i].ID)+"E"+str(j+1))
					fileout.write(line)
				filedroga.close()
				filedrogadir = open('templates/link_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"E"+str(j+1)).replace("{dir}", "E")
					line = line.replace("{width}","4").replace("{height}", "0.5")
					fileout.write(line)
				filedrogadir.close()
				filedrogadir = open('templates/link_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"E"+str(j+1)).replace("{dir}", "W")
					line = line.replace("{width}","4").replace("{height}", "0.5")
					fileout.write(line)
				filedrogadir.close()

		filedrogadir = open('templates/link_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "E")
			line = line.replace("{width}","4").replace("{height}", "0.5")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/link_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "E")
				fileout.write(line)
			filelatarnia.close()

	if crossway_list[i].neighbors[2]!="0":
	
		if numberOfRoadKafeleks!=0:
			for j in range(0,numberOfRoadKafeleks):
				filedroga = open('templates/link_droga', 'r')
				for line in filedroga:
					line = line.replace("{id}",str(crossway_list[i].ID)+"S"+str(j+1))
					fileout.write(line)
				filedroga.close()
				filedrogadir = open('templates/link_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"S"+str(j+1)).replace("{dir}", "N")
					line = line.replace("{width}","0.5").replace("{height}", "4")
					fileout.write(line)
				filedrogadir.close()
				filedrogadir = open('templates/link_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"S"+str(j+1)).replace("{dir}", "S")
					line = line.replace("{width}","0.5").replace("{height}", "4")
					fileout.write(line)
				filedrogadir.close()

		filedrogadir = open('templates/link_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "S")
			line = line.replace("{width}","0.5").replace("{height}", "4")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/link_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "S")
				fileout.write(line)
			filelatarnia.close()

	if crossway_list[i].neighbors[3]!="0":
		filedrogadir = open('templates/link_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "W")
			line = line.replace("{width}","4").replace("{height}", "0.5")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/link_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "W")
				fileout.write(line)
			filelatarnia.close()

for line in filein:
	fileout.write(line)
	if '<!-- joints  -->' in line:
		break

for i in range(0,len(crossway_list)):

	filedroga = open('templates/joint_droga', 'r')
	for line in filedroga:
		line = line.replace("{id}",str(crossway_list[i].ID))
		line = line.replace("{x}",str(crossway_list[i].x))
		line = line.replace("{y}",str(crossway_list[i].y))
		fileout.write(line)
	filedroga.close()

	connections = 0
	for j in range(0,len(crossway_list[i].neighbors)):
		if crossway_list[i].neighbors[j]!="0":
			connections=connections+1

	if crossway_list[i].neighbors[0]!="0":
		filedrogadir = open('templates/joint_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "N")
			line = line.replace("{x}","-2.25").replace("{y}", "0")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/joint_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "N")
				line = line.replace("{x}","-2.25").replace("{y}", "-2.25")
				line = line.replace("{alfa}", "1.57")
				fileout.write(line)
			filelatarnia.close()

	if crossway_list[i].neighbors[1]!="0":

		numberOfRoadKafeleks = int(crossway_list[i].lenghts[1])
		if numberOfRoadKafeleks!=0:
			for j in range(0,numberOfRoadKafeleks):
				filedroga = open('templates/joint_droga', 'r')
				for line in filedroga:
					line = line.replace("{id}",str(crossway_list[i].ID)+"E"+str(j+1))
					line = line.replace("{x}",str(crossway_list[i].x))
					line = line.replace("{y}",str(crossway_list[i].y+(j+1)*5))
					fileout.write(line)
				filedroga.close()
				filedrogadir = open('templates/joint_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"E"+str(j+1)).replace("{dir}", "E")
					line = line.replace("{x}","0").replace("{y}", "2.25")
					fileout.write(line)
				filedrogadir.close()
				filedrogadir = open('templates/joint_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"E"+str(j+1)).replace("{dir}", "W")
					line = line.replace("{x}","0").replace("{y}", "-2.25")
					fileout.write(line)
				filedrogadir.close()

		filedrogadir = open('templates/joint_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "E")
			line = line.replace("{x}","0").replace("{y}", "2.25")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/joint_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "E")
				line = line.replace("{x}","-2.25").replace("{y}", "2.25")
				line = line.replace("{alfa}", "0")
				fileout.write(line)
			filelatarnia.close()

	if crossway_list[i].neighbors[2]!="0":
		if numberOfRoadKafeleks!=0:
			for j in range(0,numberOfRoadKafeleks):
				filedroga = open('templates/joint_droga', 'r')
				for line in filedroga:
					line = line.replace("{id}",str(crossway_list[i].ID)+"S"+str(j+1))
					line = line.replace("{x}",str(crossway_list[i].x+(j+1)*5))
					line = line.replace("{y}",str(crossway_list[i].y))
					fileout.write(line)
				filedroga.close()
				filedrogadir = open('templates/joint_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"S"+str(j+1)).replace("{dir}", "N")
					line = line.replace("{x}","-2.25").replace("{y}", "0")
					fileout.write(line)
				filedrogadir.close()
				filedrogadir = open('templates/joint_droga_dir', 'r')
				for line in filedrogadir:
					line = line.replace("{id}",str(crossway_list[i].ID)+"S"+str(j+1)).replace("{dir}", "S")
					line = line.replace("{x}","2.25").replace("{y}", "0")
					fileout.write(line)
				filedrogadir.close()


		filedrogadir = open('templates/joint_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "S")
			line = line.replace("{x}","2.25").replace("{y}", "0")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/joint_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "S")
				line = line.replace("{x}","2.25").replace("{y}", "2.25")
				line = line.replace("{alfa}", "-1.57")
				fileout.write(line)
			filelatarnia.close()

	if crossway_list[i].neighbors[3]!="0":
		filedrogadir = open('templates/joint_droga_dir', 'r')
		for line in filedrogadir:
			line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "W")
			line = line.replace("{x}","0").replace("{y}", "-2.25")
			fileout.write(line)
		filedrogadir.close()
		if connections>2:
			filelatarnia = open('templates/joint_latarnia', 'r')
			for line in filelatarnia:
				line = line.replace("{id}",str(crossway_list[i].ID)).replace("{dir}", "W")
				line = line.replace("{x}","2.25").replace("{y}", "-2.25")
				line = line.replace("{alfa}", "3.14")
				fileout.write(line)
			filelatarnia.close()

for line in filein:
	fileout.write(line)

fileout.close()
filein.close()



