import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import math


class XYNode:
	g_score = 0
	f_score = 40
	parent = '00000'
	key = '00000'
	x = 1
	y = 1

class XYPlanner:

	def NodeInitialize(self,node,x_init,y_init):
		node.x = x_init
		node.y = y_init
		node.key = str(node.x)+str(node.y)
		return node

	def g_score(self,current,neighbor):
		g_score = math.sqrt((current.x-neighbor.x)**2+(current.y-neighbor.y)**2)
		return g_score

	def heuristic_cost(self,neighbor,goal):
		heuristic_cost = math.sqrt((neighbor.x-goal.x)**2+(neighbor.y-goal.y)**2)
		return heuristic_cost

	def goalReached(self,current,goal):
		if abs(goal.x-current.x) < 0.1 and abs(goal.y-current.y) < 0.1:
			return 'true'
		else:
			return 'false'

	def GetNeighbors(self,current):
		x_val = current.x
		y_val = current.y
		df = 0.5
		max_radius = 1
		neighbor_list = []

		for i in np.arange(current.x - max_radius,current.x + max_radius+df,df):
			for j in np.arange(current.y - max_radius,current.y + max_radius+df,df):
				if math.sqrt(abs(current.x-i)**2 + abs(current.y-j)**2) > 1e-8:

					neighbor_node = XYNode()
					neighbor_node.g_score = 0
					neighbor_node.x = i
					neighbor_node.y = j
					neighbor_node.key = str(neighbor_node.x)+str(neighbor_node.y)
					neighbor_node.parent = current.key
					neighbor_list.append(neighbor_node)

		return neighbor_list

	def PlotNeighbors(self,neighbor_list):
		xcoords = []
		ycoords = []
		j = 0
		for i in neighbor_list:
			print(i.key)
			xcoords.append(i.x)
			ycoords.append(i.y)
			j = j+1
		plt.plot(xcoords,ycoords,'.')
		plt.title('neighbor nodes')
		plt.xlabel('x pos')
		plt.ylabel('y pos')
		plt.show()
		#print(xcoords)
		#print(ycoords)

	def CompareClosedSet(self,ns_node,ClosedSet):
		if ns_node in ClosedSet:
			return 'true'
		else:
			return 'false'

	def CompareExploredSet(self,ns_node,ExploredSet):
		if ns_node in ExploredSet:
			return 'true'
		else:
			return 'false'



print('hello world')


def DoAstar():
	OpenSet = []
	ClosedSet = []
	ExploredSet = []

	begin_node = XYNode()
	goal_node = XYNode()
	test_node1 = XYNode()
	test_node2 = XYNode()
	planner = XYPlanner()
	begin_node = planner.NodeInitialize(begin_node,3,4)
	goal_node = planner.NodeInitialize(goal_node,5,6)
	test_node1 = planner.NodeInitialize(test_node1,3.5,4.5)
	test_node2 = planner.NodeInitialize(test_node2,3.5,4)
	test_node1.g_score = 100
	test_node1.f_score = 200
	test_node1.parent = 'blah blah'
	test_node2.g_score = 400
	test_node2.f_score = 300


	OpenSet.append(begin_node)
	#OpenSet.append(test_node1)
	ExploredSet.append(begin_node)
	ExploredSet.append(test_node1)
	ExploredSet.append(test_node2)

	#ClosedSet.append(test_node1)
	#ClosedSet.append(test_node2)





	b = 0

	while(b<3):
		OpenSet.sort(key=lambda xy: xy.f_score)
		current_node = OpenSet[0]

		print('contents of open set after sort: ')
		for oo in OpenSet:
			print('key: %s, parent: %s' %(oo.key,oo.parent))

		# print('current node: %s' %(current_node.key))
		# print('size of open set: %f' % (len(OpenSet)))
		# print('contents of sorted open set: ')
		# for mm in OpenSet:
		# 	print('key: %s gscore: %f fscore: %f' %(mm.key,mm.g_score,mm.f_score))


		if planner.goalReached(current_node,goal_node) == 'true':
			print('FOUND A PATH')
			OpenSet.pop(0)
			break
		else:		
			OpenSet.pop(0)

			ClosedSet.append(current_node)
			ExploredSet.remove(current_node)

			neighbor_nodes = planner.GetNeighbors(current_node)
			#planner.PlotNeighbors(neighbor_nodes)

			for ns_node in neighbor_nodes:
				if planner.CompareClosedSet(ns_node,ClosedSet) == 'false'
						for es_ind,es_node in enumerate(ExploredSet):

							if abs(es_node.x - ns_node.x) < 1e-5 and abs(es_node.y - ns_node.y) < 1e-5:

								print('Node already explored :D with key: %s' % (ns_node.key))

								tentative_gscore = current_node.g_score + planner.g_score(current_node,ns_node)

								if tentative_gscore < es_node.g_score:

									print('update step')
									es_node.parent = current_node.key
									es_node.g_score = tentative_gscore
									es_node.f_score = tentative_gscore + planner.heuristic_cost(ns_node,goal_node)
									

							else:
								print('adding new node to open set')
								#print('not in explored set')
								ns_node.g_score = current_node.g_score + planner.g_score(current_node,ns_node)
								ns_node.f_score = ns_node.g_score + planner.heuristic_cost(ns_node,goal_node)
								#OpenSet.append(ns_node)
								#ExploredSet.append(ns_node)

	# 						ExploredSet.append(ns_node)
	# 			print('complete iteration %i' % (index))
	# 			index = index + 1
	# 			b = b + 1
		#print('Open set size: %f' %(len(OpenSet)))				#print('new node with g score: %f and f score: %f' %(ns_node.g_score,ns_node.f_score))
		b = b+1






	



DoAstar()




