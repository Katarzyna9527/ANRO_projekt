#!/usr/bin/python
import roslaunch
import sys
import time

if len(sys.argv)<3:
	print "Nie podano argumentow. 1 to ilosc samochodow, a 2 to czas odstepu miedzy nimi."
	exit();
	
ps = []
for i in range(int(sys.argv[1])):
	package = 'cars'
	executable = 'cars_node'
	node = roslaunch.core.Node(package, executable)

	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()
	
	p = launch.launch(node)
	ps.append(p)
	time.sleep(int(sys.argv[2]));
while True:
	for i in ps:
		if not i.is_alive():
			exit()
		

