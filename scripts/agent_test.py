#!/usr/bin/env python3

from sys import path
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cbga.msg import FloatList
from cbga.msg import IntList
from math import *


class AgentNode():

    def bid_list_callback(self, data):
        self.other_agent_bid_list[data.id - 1] = data

    def cost_list_callback(self, data):
        self.other_agent_cost_list[data.id - 1] = data
        
    def path_list_callback(self, data):
        self.other_agent_path_list[data.id - 1] = data    


    def path_finding(self):
        #return True

        blocked_tasks = set() #List of blocked tasks IDs

        for i in range(self.num_tasks):
            if (self.check_list(self.agent_list[self.agent_index - 1].get('module'), 'wifi') and
                not self.check_list(self.agent_list[self.agent_index - 1].get('module'), 'bluetooth')):
                if not self.task_list[i].get('type') == 'wifi':
                    blocked_tasks.add(i + 1)
            elif (self.check_list(self.agent_list[self.agent_index - 1].get('module'), 'bluetooth') and
                not self.check_list(self.agent_list[self.agent_index - 1].get('module'), 'wifi')):
                if not self.task_list[i].get('type') == 'bluetooth':
                    blocked_tasks.add(i + 1)
                
        for i in range(self.num_tasks):
            if self.bid_list[i] != 0:
                blocked_tasks.add(i + 1) #Adding blocked tasks IDs 

        for i in range(self.max_depth):
            if self.path_list[i] == 0:
                #self.dist_list = [self.distance(self.task_list[k].get('location')[0], self.task_list[k].get('location')[1]) for k in range(self.num_tasks)]
                
                for j in range(self.num_tasks):
                    # cost calculation
                    cost = self.distance(self.task_list[j].get('location')[0], self.task_list[j].get('location')[1]) \
                        / self.agent_list[self.agent_index - 1].get('speed')
                    
                    if self.task_list[j].get('type') == 'bluetooth':
                        cost = cost + self.task_list[j].get('duration') * self.bluetooth_modifier
                    else:
                        cost = cost + self.task_list[j].get('duration') 

                    self.dist_list[j] = cost

                    if j + 1 in blocked_tasks: # j + 1
                        self.dist_list[j] = -1

                print("Agent_",self.agent_index," dist list: ", self.dist_list)

                min_dist_list = []
                for j in range(self.num_tasks):
                    if self.dist_list[j] != -1:
                        min_dist_list.append(self.dist_list[j])
                    min_dist_list_sorted = sorted(min_dist_list)            
                    if self.check_list(min_dist_list_sorted, 0):
                        min_dist_list_sorted[0] = max(self.dist_list) + 1
                    if len(min_dist_list_sorted) != 0:    
                        min_dist = min(min_dist_list_sorted)                
                if len(min_dist_list) == 0:
                    break
                min_dist_index = self.dist_list.index(min_dist)
                self.path_list[i] = min_dist_index + 1

                if min_dist == 0:
                    print("Min Dinstance = 0")  
                  
                self.cost_list[i] = self.cost_list[i-1] + cost           
                
                self.agent_position.x = self.task_list[min_dist_index].get('location')[0]
                self.agent_position.y = self.task_list[min_dist_index].get('location')[1]
                blocked_tasks.add(min_dist_index + 1) # min_dist_index + 1

        # Filling the bid list
        for i in range(self.num_tasks):
            if self.check_list(self.path_list, i + 1):
                self.bid_list[i] = 1    


    def compare_cost(self):

        old_index = 0
        first_pass = True
        
        for i in range(self.num_agents):  # compare with other agents 
            
            if self.other_agent_path_list[i].id != self.agent_index:
                
                for j in range(self.max_depth): # loop to check if agents have similar tasks
                    
                    agent_path_id = self.other_agent_path_list[i].list[j]

                    #print("Agent path: " + str(agent_path_id))
                    #print("Diff: ", self.other_agent_path_list[i].id, " ", self.agent_index)
                    #print("Other: ", self.other_agent_path_list)
                    if self.check_list(self.path_list, agent_path_id) and agent_path_id != 0: # the check for task similarity 
                        
                        print("Other agent path: " + str(agent_path_id) + " from Agent_" + str(i + 1))

                        this_agent_index = self.path_list.index(agent_path_id)
                        this_agent_cost = self.cost_list[this_agent_index]

                        other_agent_index = self.other_agent_path_list[i].list.index(agent_path_id)
                        other_agent_cost = self.other_agent_cost_list[i].list[other_agent_index] 
                        
                        bid_list_index = int(agent_path_id - 1)

                        if this_agent_cost > other_agent_cost:
                            #print("Ušlo se u if")
                            #print("This agent path index: " + str(this_agent_index))
                            self.agent_position.x = self.task_list[this_agent_index].get('location')[0]
                            self.agent_position.y = self.task_list[this_agent_index].get('location')[1]
                            
                            for k in range(self.max_depth - this_agent_index):
                                bid_index = self.path_list[k + this_agent_index]
                                self.path_list[k + this_agent_index] = 0
                                self.cost_list[k + this_agent_index] = 0
                                self.bid_list[bid_index - 1] = 0
                                #print("Agent_", self.agent_index, " k: ",k, ", bid list index: ",bid_index - 1)                                 
                            
                            if max(self.path_list) == 0:
                                self.agent_position.x = self.agent_list[self.agent_index - 1].get('location')[0]
                                self.agent_position.y = self.agent_list[self.agent_index - 1].get('location')[1]
                                print("Returned Agent_", self.agent_index,"to position:",self.agent_position.x,",",self.agent_position.y)

                            else:
                                for n in range(self.max_depth):
                                    if self.path_list[n] == 0:
                                        self.agent_position.x = self.task_list[self.path_list[n - 1] - 1].get('location')[0]
                                        self.agent_position.y = self.task_list[self.path_list[n - 1] - 1].get('location')[1]
                                        print("Returned Agent_", self.agent_index,"to position:",self.agent_position.x,",",self.agent_position.y)
                                        break
                            
                            if first_pass:
                                self.bid_list[bid_list_index] = -1
                                first_pass = False
                                old_index = bid_list_index
                            else:
                                self.bid_list[bid_list_index] = -1
                                self.bid_list[old_index] = 0
                                old_index = bid_index   


                        elif this_agent_cost == other_agent_cost:
                            rospy.signal_shutdown("2 similar Costs, cant converge")
                            rospy.loginfo("2 similar Costs, cant converge. Agent_" + str(self.agent_index) + " cost: " + str(this_agent_cost) + " other agent cost: " + str(other_agent_cost))        
                        

    def run(self):
        
        counter = 0
        old_list = [0 for _ in range(self.max_depth)]

        while not rospy.is_shutdown():
            self.agent_bid_list_pub.publish(self.bid_list, self.agent_index)
            self.agent_cost_list_pub.publish(self.cost_list, self.agent_index)
            self.agent_path_list_pub.publish(self.path_list, self.agent_index)
            # Publish new data every n seconds 
            rospy.sleep(1)

            print("Start Agent_" + str(self.agent_index))

            self.path_finding()

            print("Agent_", self.agent_index, " planed path: ", self.path_list)

            #rospy.sleep(2)

            self.compare_cost()

            #rospy.sleep(2)
            print("Agent_" + str(self.agent_index) + " cost list: " + str(self.cost_list))
            print("Agent_", self.agent_index, "bid list: ", self.bid_list)
            print("Agent_" + str(self.agent_index) + " old path list: " + str(old_list))

            if old_list == self.path_list:
                counter += 1
                old_list = self.path_list.copy()
            else:
                counter = 0
                old_list = self.path_list.copy()
            print("Agent_" + str(self.agent_index) + " path list: " + str(self.path_list))
            print("Counter = ", counter)
            if counter > self.consensus_counter:
                self.finished_pub.publish("Anget_" + str(self.agent_index))
                rospy.signal_shutdown("Optimal route found.")
                rospy.loginfo("Optimal route found. Shuting down Agent_" + str(self.agent_index))
                
            print("End Agent_" + str(self.agent_index))    




    def distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.agent_position.x), 2) + pow((goal_y - self.agent_position.y), 2))
        return distance

    
    def check_list(self, list1, val):
        """
        compare if a value is equal to an element in a list 
        """
        for x in list1:
            if val == x:
                return True 
        return False


    def settings(self):
        
        """
        Initialize some lists given new AgentList and TaskList
        """

        self.num_agents = len(self.agent_list)
        self.num_tasks = len(self.task_list)

        # initialize these properties
        # Bundle list (first n tasks)
        self.bundle_list = [self.task_list[i].get('id') for i in range(self.max_depth)]

        # Path lists (n closest task to the agent)
        self.path_list = [0 for _ in range(self.max_depth)]
        self.other_agent_path_list = [0 for _ in range(self.num_agents)]
        
        """
        #Times list
        self.times_list = [0 for _ in range(self.max_depth)]
        """

        # Cost lists (list of Costs for each task) <<< MOST IMPORTANT
        self.cost_list = [0 for _ in range(self.max_depth)] 
        self.other_agent_cost_list = [0 for _ in range(self.num_agents)]

        # Bid lists (1-biding for the task, 0-not biding for the task, -1-lost the task) <<< MOST IMPORTANT
        self.bid_list = [0 for _ in range(self.num_tasks)]
        self.other_agent_bid_list = [0 for _ in range(self.num_agents)]

        """
        # fixed the initialization, from 0 vector to -1 vector
        self.winners_list = [-1 for _ in range(self.num_tasks)]
        self.winner_bid_list = [-1 for _ in range(self.num_tasks)]
        """

        # Dist list
        self.dist_list = [0 for _ in range(self.num_tasks)]

        

    def __init__(self):

        self.agent_list = rospy.get_param('agents')
        self.task_list = rospy.get_param('tasks')

        self.agent_index = int (rospy.get_name().partition('_')[2])
        
        self.agent_position = Point()
        self.agent_position.x = self.agent_list[self.agent_index - 1].get('location')[0]
        self.agent_position.y = self.agent_list[self.agent_index - 1].get('location')[1]

        # depth of the bundle (n tasks per bundle)
        self.max_depth = 4

        # bluetooth modifier
        self.bluetooth_modifier = 2

        # Number of repetitions of same cost required to determine consensus
        self.consensus_counter = 20

        # All the necessary lists 
        self.settings()

        # Bid list publisher
        #self.agent_bid_list_pub = rospy.Publisher(str('Agent_' + str(self.agent_index) + '/bid_list'), FloatList, queue_size=1)
        self.agent_bid_list_pub = rospy.Publisher('/bid_list', IntList, queue_size = 1)

        # Cost list publisher 
        #self.agent_cost_list_pub = rospy.Publisher(str('Agent_' + str(self.agent_index) + '/cost_list'), FloatList, queue_size=1)
        self.agent_cost_list_pub = rospy.Publisher('/cost_list', FloatList, queue_size = 1)

        # Path list publisher
        self.agent_path_list_pub = rospy.Publisher('/path_list', IntList, queue_size = 1)

        # Finished publisher
        self.finished_pub = rospy.Publisher('/finished_agents', String, queue_size = 1)

        # Subscribers

        rospy.Subscriber('/bid_list', IntList, self.bid_list_callback)
        rospy.Subscriber('/cost_list', FloatList, self.cost_list_callback)
        rospy.Subscriber('/path_list', IntList, self.path_list_callback)

        print("Agent_",self.agent_index," path list: ")
        print(self.path_list)
        
        print("Agent_",self.agent_index," cost list:")
        print(self.cost_list)
        
        print("Agent_",self.agent_index," bid list:")
        print(self.bid_list)

        while (
            self.agent_bid_list_pub.get_num_connections() < self.num_agents and 
            self.agent_cost_list_pub.get_num_connections() < self.num_agents and 
            self.agent_path_list_pub.get_num_connections() < self.num_agents + 1 and
            not rospy.is_shutdown()
            ):
            rospy.loginfo("Waiting for subscriber to connect")
            rospy.sleep(1)

        rospy.loginfo('Num of conections for bid list: ' + str(self.agent_bid_list_pub.get_num_connections()))
        rospy.loginfo('Num of conections for cost list: ' + str(self.agent_cost_list_pub.get_num_connections()))
        rospy.loginfo('Num of conections for path list: ' + str(self.agent_path_list_pub.get_num_connections()))
        rospy.loginfo(rospy.get_name())

        

if __name__ == '__main__':
    rospy.init_node('Agent_1')
    
    try:
        ne = AgentNode()
        ne.run()
    except rospy.ROSInterruptException: pass 
    
    

        
    
    



