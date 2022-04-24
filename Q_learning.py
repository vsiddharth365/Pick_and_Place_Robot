import math
import os
import random
import sys
import matplotlib.pyplot as plt
import pybullet as p
from gripper import liftUp, loadWorld, loadGripper, reachXY, lowerDown, Gripper, lock, pick

rows, cols = 30, 30
loadWorld()
gripper = loadGripper()


def q_learning():
    battery = 100
    discharge_rate = 3
    alpha = 0.03
    # learning rate
    gamma = 0.5
    # discount parameter
    arr, ipos = initial_state()
    for i in range(len(ipos)):
        p.loadURDF(os.path.join(os.getcwd(), "cube_"+str(i+1)+".urdf"), [0.25+0.5*ipos[i][0], 0.25+0.5*ipos[i][1], 0])
    for i in range(-1, 31):
        p.loadURDF(os.path.join(os.getcwd(), "cube_small.urdf"), [0.25 + 0.5 * i, -0.25, 0])
    for i in range(0, 31):
        p.loadURDF(os.path.join(os.getcwd(), "cube_small.urdf"), [-0.25, 0.25 + 0.5 * i, 0])
    for i in range(0, 30):
        p.loadURDF(os.path.join(os.getcwd(), "cube_small.urdf"), [15.25, 0.25 + 0.5 * i, 0])
    for i in range(0, 31):
        p.loadURDF(os.path.join(os.getcwd(), "cube_small.urdf"), [0.25 + 0.5 * i, 15.25, 0])
    objects = list_of_objects()
    print("Initial environment :")
    print_array(arr)
    goal_arr, fpos = final_state()
    q_value, nsr_value, j = [], [], 0
    # Initializing Q-table and N-step return values with 0
    for _ in range(0, len(objects)):
        q_value.append([0.0])
        nsr_value.append([0.0])
    # index lists to choose better actions ahead
    index8, index5, index3 = [8], [5], [3]
    it, bat, flag, warning = [], [], 0, 0
    for i in range(0, 10):
        x = ipos[i][0]
        y = ipos[i][1]
        a = fpos[i][0]
        b = fpos[i][1]
        pick(0.25+0.5*x, 0.25+0.5*y, gripper)
        arr[x][y] = 0
        q, nsr, k = 0, 0, 0
        while not (x == a and y == b):
            distance, pos = next_state(a, b, x, y)
            # Sorting distance[] and swapping pos[] accordingly
            for _ in range(0, len(distance)):
                for __ in range(_ + 1, len(distance)):
                    if distance[__] < distance[_]:
                        distance[_], distance[__] = distance[__], distance[_]
                        pos[_], pos[__] = pos[__], pos[_]
            reward = 0
            # Choosing better random direction out of <=8 available to maximize reward obtained for next action
            if len(distance) == 8:
                if len(set(index8)) >= 1 / (10 * alpha) + 1:
                    chosen = random.randint(0, min(index8) - 1)
                else:
                    chosen = random.randint(0, 7)
            elif len(distance) == 5:
                if len(set(index5)) >= 1 / (20 * alpha) + 1:
                    chosen = random.randint(0, min(index5) - 1)
                else:
                    chosen = random.randint(0, 4)
            else:
                if len(set(index3)) >= 1 / (30 * alpha) + 1:
                    chosen = random.randint(0, min(index3) - 1)
                else:
                    chosen = random.randint(0, 2)
            # Computing the required amount of battery to place the object at its final position from current position
            required = math.ceil(distance[chosen]) + discharge_rate
            if battery < required:
                if flag:
                    # If warnings' limit has been reached, robot has learnt to avoid lower battery levels than required
                    battery = required
                    reward = 1
                else:
                    # If robot does not recharge when battery is less than required, deduct reward and give another warning
                    reward = -1
                    warning += 1
                    if battery == 0:
                        # If battery reaches 0%, then object falls down and gets damaged. Hence, greater penalty on reward is given
                        reward = -4
                        warning += 1
                        # Robot has to be manually charged after yet another warning
                        battery = required
            if not flag and warning >= 1 / (10 * alpha):
                # If learning rate (alpha) is higher, the lesser warnings are required to make robot learn to avoid lower battery levels
                flag = 1
            # Executing the action
            if pos[chosen] == 0:
                x -= 1
                y -= 1
            elif pos[chosen] == 1:
                x -= 1
            elif pos[chosen] == 2:
                x -= 1
                y += 1
            elif pos[chosen] == 3:
                y -= 1
            elif pos[chosen] == 4:
                y += 1
            elif pos[chosen] == 5:
                x += 1
                y -= 1
            elif pos[chosen] == 6:
                x += 1
            elif pos[chosen] == 7:
                x += 1
                y += 1
            reachXY(0.25+0.5*x, 0.25+0.5*y, 0.02, 0.13, gripper)
            # Storing current battery level
            bat.append(battery)
            # Storing current episode number
            it.append(j)
            j += 1
            # battery drains by an specified rate at each episode
            battery = max(battery - discharge_rate, 0)
            # reward is a fraction
            reward += (len(distance) - chosen) / len(distance)
            minimum_reward, worst_possible_chosen_index = 1 / 8, 7
            # If the object reaches its correct position, increase the reward by 5 times
            if x == a and y == b:
                reward *= 5
            else:
                if chosen:
                    if len(distance) == 8:
                        index8.append(chosen)
                    elif len(distance) == 5:
                        index5.append(chosen)
                    else:
                        index3.append(chosen)
                # Finding worst action which will yield minimum reward in next state, based on current state
                if (x == 0 and y == 0) or (x == 0 and y == 29) or (x == 29 and y == 0) or (x == 29 and y == 29):
                    if len(set(index3)) >= 1 / (30 * alpha) + 1:
                        worst_possible_chosen_index = min(index3) - 1
                    else:
                        worst_possible_chosen_index = 2
                    minimum_reward = (3 - worst_possible_chosen_index) / 3
                elif ((0 < y < 29) and (x == 0 or x == 29)) or ((0 < x < 29) and (y == 0 or y == 29)):
                    if len(set(index5)) >= 1 / (20 * alpha) + 1:
                        worst_possible_chosen_index = min(index5) - 1
                    else:
                        worst_possible_chosen_index = 4
                    minimum_reward = (5 - worst_possible_chosen_index) / 5
                elif 0 < x < 29 and 0 < y < 29:
                    if len(set(index8)) >= 1 / (10 * alpha) + 1:
                        worst_possible_chosen_index = min(index8) - 1
                    else:
                        worst_possible_chosen_index = 7
                    minimum_reward = (8 - worst_possible_chosen_index) / 8
                distance = next_state(a, b, x, y)[0]
                distance.sort()
                # Computing minimum reward obtained in next state due to battery level, the worst possible reward will
                # occur at worst_possible_chosen_index
                required = math.ceil(distance[worst_possible_chosen_index]) + discharge_rate
                if battery < required:
                    if flag:
                        minimum_reward += 1
                    else:
                        minimum_reward -= 1
                        if battery == 0:
                            minimum_reward -= 4
                # Defining q(s,a) for current state-action pair
                q = reward
                # q(s',a') is the q-value of next state-action pair
                # Since q(s',a') = (q(s,q) - reward)/gamma, maximum value max_q of q(s',a') = (q(s,a) - (minimum reward))/gamma
                max_q = (q - minimum_reward) / gamma
                # Defined equation for Q-learning algorithm
                q = (1 - alpha) * q + alpha * (reward + gamma * max_q)
                # Computing N-step returns for ith object
                nsr += (gamma ** k) * reward
                k += 1
                # Storing q(s,a) values for ith object
                q_value[i].append(q)
                # Storing N-step returns for ith object
                nsr_value[i].append(nsr)
        lowerDown(0.02, gripper)
        lock(0, gripper)
        Gripper(0, gripper)
        liftUp(0, gripper)
        arr[x][y] = i + 1
    print("\nFinal environment :")
    print_array(arr)
    print("\nDesired goal state :")
    print_array(goal_arr)
    if arr == goal_arr:
        print("\nGoal state achieved.")
    efficiency = []
    for i in range(0, len(objects)):
        steps_taken = len(q_value[i])
        initial_distance = euclidean(fpos[i][0], fpos[i][1], ipos[i][0], ipos[i][1])
        efficiency.append(initial_distance / steps_taken)
    m = max(efficiency)
    for i in range(0, len(objects)):
        efficiency[i] = 100 * (efficiency[i] / m)
    average_efficiency = sum(efficiency) / len(efficiency)
    plt.figure(1)
    plt.title("Q-learning")
    print("\nQ-values for placing each object on its correct position :")
    markers = ['*', '^', 'v', 's', 'o', 'p', 'h', 'd', 'P', '.']
    for i in range(0, len(objects)):
        print(q_value[i])
        lis = []
        for j in range(0, len(q_value[i])):
            lis.append(j)
        marker = markers[i]
        plt.scatter(lis, q_value[i], marker=marker)
        plt.plot(lis, q_value[i], label=f"Object {i + 1}")
    plt.xlabel("Episodes")
    plt.ylabel("Q values")
    plt.legend()
    plt.figure(2)
    plt.title("N-Step Return Values")
    print("\nN-Step return values for placing each object at its correct position :")
    for i in range(0, len(objects)):
        print(nsr_value[i])
        lis = []
        for j in range(0, len(nsr_value[i])):
            lis.append(j)
        marker = markers[i]
        plt.scatter(lis, nsr_value[i], marker=marker)
        plt.plot(lis, nsr_value[i], label=f"Object {i + 1}")
    plt.xlabel("Episodes")
    plt.ylabel("N-Step Return")
    plt.legend()
    lis = []
    for i in range(len(objects)):
        lis.append(str(objects[i]))
    plt.figure(3)
    plt.bar(lis, efficiency, color='green', width=0.35)
    plt.xlabel("Objects")
    plt.ylabel("Efficiency")
    plt.title("Efficiency achieved in arranging each object")
    plt.grid(True)
    print("\nThe efficiency (in percent) of placing each object at its right position :")
    print(efficiency)
    print(f"\nAverage efficiency : {average_efficiency}%\n")
    f.close()
    plt.figure(4)
    plt.title("Battery levels at each episode")
    plt.xlabel("Episodes")
    plt.ylabel("Battery level")
    plt.bar(it, bat, color='blue', width=0.25)
    plt.show()


def next_state(a, b, x, y):
    """
    0 1 2
    3 X 4     X is the current position with next possible positions 0 to 7
    5 6 7
    """
    distance, pos = [], []
    # Calculating distances between location of each next possible state and the final location
    if x - 1 >= 0 and y - 1 >= 0:
        distance.append(euclidean(a, b, x - 1, y - 1))
        pos.append(0)
    if x - 1 >= 0:
        distance.append(euclidean(a, b, x - 1, y))
        pos.append(1)
    if x - 1 >= 0 and y + 1 <= 29:
        distance.append(euclidean(a, b, x - 1, y + 1))
        pos.append(2)
    if y - 1 >= 0:
        distance.append(euclidean(a, b, x, y - 1))
        pos.append(3)
    if y + 1 <= 29:
        distance.append(euclidean(a, b, x, y + 1))
        pos.append(4)
    if x + 1 <= 29 and y - 1 >= 0:
        distance.append(euclidean(a, b, x + 1, y - 1))
        pos.append(5)
    if x + 1 <= 29:
        distance.append(euclidean(a, b, x + 1, y))
        pos.append(6)
    if x + 1 <= 29 and y + 1 <= 29:
        distance.append(euclidean(a, b, x + 1, y + 1))
        pos.append(7)
    return [distance, pos]


# Utility function to print a 2D matrix
def print_array(arr):
    for i in arr:
        for j in i:
            print(j, end=" ")
        print()


# Euclidean distance between current position and goal position
def euclidean(a, b, x, y):
    return math.sqrt((a - x) ** 2 + (b - y) ** 2)


# Objects labeled 1 to m, m=10
def list_of_objects():
    objects = []
    for i in range(0, 10):
        objects.append(i + 1)
    return objects


# Objects misplaced in nXn array, 2*m<n*n, n=30
def initial_state():
    init_arr = [[0 for _ in range(cols)] for __ in range(rows)]
    position = []
    x = random.randint(0, 29)
    y = random.randint(0, 29)
    for i in range(0, 10):
        while init_arr[x][y] != 0:
            x = random.randint(0, 29)
            y = random.randint(0, 29)
        init_arr[x][y] = i + 1
        position.append([x, y])
    return init_arr, position


# Randomly decided final correct positions of m=10 objects
def final_state():
    init_arr = initial_state()[0]
    goal_arr = [[0 for _ in range(cols)] for __ in range(rows)]
    position = []
    x, y = [], []
    for i in range(len(goal_arr[0])):
        for j in range(len(goal_arr[0])):
            if init_arr[i][j] == 0:
                x.append(i)
                y.append(j)
    for i in range(0, len(list_of_objects())):
        j = random.randint(0, len(x) - 1)
        # (x,y) is the ordered pair of indices such that goal_arr[x][y] = 0
        position.append([x[j], y[j]])
        goal_arr[x[j]][y[j]] = i + 1
        x.pop(j)
        y.pop(j)
    return goal_arr, position


if __name__ == "__main__":
    with open("Q-learning.txt", "w") as f:
        sys.stdout = f
        q_learning()
