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
		max_radius = 2
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
		for cs_node in ClosedSet:
			if abs(ns_node.x-cs_node.x) < 1e-5 and abs(ns_node.y-cs_node.y) < 1e-5:
				return 'true'


	def CompareExploredSet(self,ns_node,ExploredSet):
		for es_node in ExploredSet:
			if abs(ns_node.x-es_node.x) < 1e-5 and abs(ns_node.y-es_node.y) < 1e-5:
				return es_node


	def RemoveFromExploredSet(self,current_node,ExploredSet):
		for es_node in ExploredSet:
			if abs(current_node.x-es_node.x) < 1e-5 and abs(current_node.y-es_node.y) < 1e-5:
				ExploredSet.remove(es_node)
			




def DoAstar():
	OpenSet = []
	ClosedSet = []
	ExploredSet = []

	begin_node = XYNode()
	goal_node = XYNode()
	test_node1 = XYNode()
	test_node2 = XYNode()
	planner = XYPlanner()
	begin_node = planner.NodeInitialize(begin_node,1,1)
	goal_node = planner.NodeInitialize(goal_node,12.5,21)
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

	ClosedSet.append(begin_node)
	ClosedSet.append(test_node1)
	ClosedSet.append(test_node2)



	while(OpenSet):

		OpenSet.sort(key=lambda xy: xy.f_score)
		current_node = OpenSet[0]

		print('current node: %s' %(current_node.key))


		if planner.goalReached(current_node,goal_node) == 'true':
			print('FOUND A PATH')
			OpenSet.pop(0)
			break
		else:		
			OpenSet.pop(0)

			ClosedSet.append(current_node)
			planner.RemoveFromExploredSet(current_node,ExploredSet)

			neighbor_nodes = planner.GetNeighbors(current_node)

			for ns_node in neighbor_nodes:
				if planner.CompareClosedSet(ns_node,ClosedSet) != 'true':
					es_node = planner.CompareExploredSet(ns_node,ExploredSet)
					if es_node:

						tentative_gscore = current_node.g_score + planner.g_score(current_node,ns_node)

						if tentative_gscore < es_node.g_score:

							es_node.parent = current_node.key
							es_node.g_score = tentative_gscore
							es_node.f_score = tentative_gscore + planner.heuristic_cost(ns_node,goal_node)
									
					else:
						ns_node.g_score = current_node.g_score + planner.g_score(current_node,ns_node)
						ns_node.f_score = ns_node.g_score + planner.heuristic_cost(ns_node,goal_node)
						OpenSet.append(ns_node)
						ExploredSet.append(ns_node)



DoAstar()





