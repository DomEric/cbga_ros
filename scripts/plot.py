#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cbga.msg import IntList
from cbga.msg import FloatList


class PlotNode():


    def path_list_callback(self, data):
        self.path_list[data.id - 1] = data


    def finished_callback(self, data):
        self.finished_agents.append(data)


    def cost_list_callback(self, data):
        self.cost_list[data.id - 1] = data


    def run(self):

        while not rospy.is_shutdown():

            while self.path_list[0] == 0 or \
                self.path_list[1] == 0 or \
                self.path_list[2] == 0 or \
                self.path_list[3] == 0:
                rospy.sleep(1)

            while len(self.finished_agents) < self.num_agents and not rospy.is_shutdown():
                
                plt.figure(1)

                tasks_x = [self.task_list[i].get('location')[0] for i in range(self.num_tasks)] 
                tasks_y = [self.task_list[i].get('location')[1] for i in range(self.num_tasks)]
                plt.plot(tasks_x, tasks_y, 'ro', label = 'Tasks')
                i = 0
                for i_x, i_y in zip(tasks_x, tasks_y):
                    i += 1
                    plt.text(i_x, i_y, '({}, {})'.format(i_x, i_y))
                    plt.text(i_x, i_y + 2, 'Task {}'.format(i))

                agent_x = [self.agent_list[i].get('location')[0] for i in range(self.num_agents)]
                agent_y = [self.agent_list[i].get('location')[1] for i in range(self.num_agents)] 
                plt.plot(agent_x, agent_y, 'bs', label = 'Agent starting position')
                i = 0
                for i_x, i_y in zip(agent_x, agent_y):
                    i += 1
                    plt.text(i_x, i_y, '({}, {})'.format(i_x, i_y))
                    plt.text(i_x, i_y + 2, 'Agent {}'.format(i))

                for i in range(self.num_agents):
                        
                    agent_id = self.path_list[i].id - 1
                    plot_list_x = [self.agent_list[agent_id].get('location')[0]]
                    plot_list_y = [self.agent_list[agent_id].get('location')[1]]

                    for j in range(len(self.path_list[0].list)):

                        if self.path_list[i].list[j] != 0:
                            task_index = int(self.path_list[i].list[j] - 1)
                            plot_list_x.append(self.task_list[task_index].get('location')[0])
                            plot_list_y.append(self.task_list[task_index].get('location')[1])                    

                    if self.path_list[i].id == 1:
                        plt.plot(plot_list_x, plot_list_y, 'g-', label = 'Agent 1 path')
                    elif self.path_list[i].id == 2:
                        plt.plot(plot_list_x, plot_list_y, 'm-', label = 'Agent 2 path')
                    elif self.path_list[i].id == 3:
                        plt.plot(plot_list_x, plot_list_y, 'c-', label = 'Agent 3 path')    
                    elif self.path_list[i].id == 4:    
                        plt.plot(plot_list_x, plot_list_y, 'y-', label = 'Agent 4 path')
                
                plt.draw()
                plt.legend()
                plt.show()

                plt.figure(2)

                agents = ['Agent ' + str(i + 1) for i in range(self.num_agents)]
                agents_index = [i for i in range(self.num_agents)]
                costs_per_agents = []
                task_per_agent = []

                for i in range(len(self.path_list[0].list)):
                    costs_per_agents.append([self.cost_list[j].list[i] for j in range(self.num_agents)])
                    task_per_agent.append([self.path_list[j].list[i] for j in range(self.num_agents)])

                for i in range(len(self.path_list[0].list)):
                    self.bar_plot = plt.barh(agents, costs_per_agents[i], color = 'magenta', edgecolor='black', alpha = 0.4)

                for i in range(len(self.path_list[0].list)):
                    for i_x, i_y in zip(agents_index, costs_per_agents[i]):
                        if task_per_agent[i][i_x] != 0:
                            plt.text(i_y - 45, i_x, 'Task_{}\ncost: {}'.format((task_per_agent[i][i_x]), round(costs_per_agents[i][i_x])))

                plt.xlabel('Vrijeme potrebno za odrađivanje zadataka u sekundama')
                plt.draw()
                plt.show()

                print("Costs per agents",costs_per_agents)

                if len(self.finished_agents) == self.num_agents:
                    rospy.signal_shutdown('Shutdown PlotNode')                             


    def __init__(self):
    
        self.agent_list = rospy.get_param('agents')
        self.task_list = rospy.get_param('tasks')

        self.num_agents = len(self.agent_list)
        self.num_tasks = len(self.task_list)

        self.agent_position = Point()
        self.path_list = [0 for _ in range(self.num_agents)]
        self.cost_list = [0 for _ in range(self.num_agents)]
        self.finished_agents = []

        rospy.Subscriber('/path_list', IntList, self.path_list_callback)
        rospy.Subscriber('/finished_agents', String, self.finished_callback)
        rospy.Subscriber('/cost_list', FloatList, self.cost_list_callback)


if __name__ == '__main__':
    rospy.init_node('MatPlot')

    try:
        ne = PlotNode()
        ne.run()
    except rospy.ROSInterruptException: pass 
         