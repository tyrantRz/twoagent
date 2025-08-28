# EXAMPLE 1 - Task Description: Wash the fork. 
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Wash the Fork. (Skills Required: GoToObject, PickupObject, PutObject, SwitchOn, CleanObject, SwitchOff)
# We can perform SubTask 1 

# CODE
def wash_fork():
    # 0: SubTask 1: Wash the Fork
    # 1: Go to the Fork.
    GoToObject('Fork')
    # 2: Pick up the Fork.
    PickupObject('Fork')
    # 3: Go to the Sink.
    GoToObject('Sink')
    # 4: Put the Fork inside the Sink.
    PutObject('Fork', 'Sink')
    # 5: Switch on the Faucet.
    SwitchOn('Faucet')
    # 6: Perform the actual cleaning exactly once.
    CleanObject('Fork')
    # 7: Dwell briefly to simulate washing.
    time.sleep(3)
    # 8: Switch off the Faucet.
    SwitchOff('Faucet')
    # 9: Pick the Fork back.
    PickupObject('Fork')
    # 10: Place it on the CounterTop (tidy up).
    GoToObject('CounterTop')
    PutObject('Fork', 'CounterTop')

# Perform SubTask 1
task1_thread = threading.Thread(target=wash_fork)
# Start executing SubTask 1 
task1_thread.start()
# Task wash the fork is done

# TASK ALLOCATION
robots = [{
    'name': 'robot1',
    'skills': [
        'GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject',
        'PickupObject', 'PutObject', 'SwitchOn', 'CleanObject', 'SwitchOff',
        'ThrowObject', 'PushObject', 'PullObject'
    ],
    'mass': 2
}]
# SOLUTION
# All the robots DONOT share the same set and number (no_skills) of skills & all objects have different masses. In this case where all robots have different sets of skills and objects have different mass - Focus on Task Allocation based on Robot Skills alone. 
# Analyze the skills required for each subtask and the skills each robot possesses. In this scenario, we have one main subtasks: 'Wash the Fork'.
# For the 'Wash the Fork' subtask, it requires 'GoToObject', 'PickupObject', 'PutObject', 'SwitchOn', and 'SwitchOff' skills. In this case, Robot 2 has all these skills.  
# No teams are required since SubTasks can be performed with individual robots as explained above. The 'Wash the Fork' subtask is assigned Robot 2. 

# Code Solution 
def wash_fork(robot_list):
    # robot_list = [robot1]
    r = robot_list[0]
    # 0: SubTask 2: Wash the Fork
    # 1: Go to the Fork using robot1.
    GoToObject(r, 'Fork')
    # 2: Pick up the Fork using robot1.
    PickupObject(r, 'Fork')
    # 3: Go to the Sink using robot1.
    GoToObject(r, 'Sink')
    # 4: Put the Fork inside the Sink using robot1.
    PutObject(r, 'Fork', 'Sink')
    # 5: Switch on the Faucet using robot1.
    SwitchOn(r, 'Faucet')
    # 6: Clean the Fork once using robot1 (critical).
    CleanObject(r, 'Fork')
    # 7: Dwell briefly to simulate washing.
    time.sleep(3)
    # 8: Switch off the Faucet using robot1.
    SwitchOff(r, 'Faucet')
    # 9: Pick the Fork back using robot1.
    PickupObject(r, 'Fork')
    # 10: Place it on the CounterTop using robot1.
    GoToObject(r, 'CounterTop')
    PutObject(r, 'Fork', 'CounterTop')

# Perform SubTask 1 with robot1
wash_fork([robots[0]])
# Task wash the fork is done


# EXAMPLE 2 - Task Description: Put tomato in fridge 
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Put Tomato in Fridge. (Skills Required: GoToObject, PickupObject, OpenObject, PutObject, CloseObject)
# We can perform SubTask 1.

# CODE
def put_tomato_in_fridge(robot_list):
    # robot_list = [robot1]
    r = robot_list[0]
    # 0: SubTask 1: Put Tomato in Fridge
    # 1: Go to the Tomato using robot1.
    GoToObject(r, 'Tomato')
    # 2: Pick up the Tomato using robot1.
    PickupObject(r, 'Tomato')
    # 3: Go to the Fridge using robot1.
    GoToObject(r, 'Fridge')
    # 4: Open the Fridge using robot1.
    OpenObject(r, 'Fridge')
    # 5: Put the Tomato in the Fridge using robot1.
    PutObject(r, 'Tomato', 'Fridge')
    # 6: Close the Fridge using robot1.
    CloseObject(r, 'Fridge')
# Perform SubTask 1 
put_tomato_in_fridge([robots[0]])
# Perform SubTask 1
task1_thread = threading.Thread(target=put_tomato_in_fridge)
# Start executing SubTask 1
task1_thread.start()
# Task Put tomato in fridge is done

# TASK ALLOCATION
robots = [{'name': 'robot1', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject', 'PickupObject', 'PutObject', 'SwitchOn', 'SwitchOff', 'DropHandObject', 'ThrowObject', 'PushObject', 'PullObject'],'mass': 4}, {'name': 'robot2', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject', 'PickupObject', 'PutObject', 'SwitchOn', 'SwitchOff', 'DropHandObject', 'ThrowObject', 'PushObject', 'PullObject'],'mass': 1}, {'name': 'robot3', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject', 'PickupObject', 'PutObject', 'SwitchOn', 'SwitchOff', 'DropHandObject', 'ThrowObject', 'PushObject', 'PullObject'],'mass': 2}]
# SOLUTION
# All the robots share the same set and number of skills (no_skills) & all objects DONOT have same mass. In this case where all objects have different mass, and robots have same sets of skills- Focus on Task Allocation based on Mass alone. 
# Analyze the mass required for each object being PickedUp by the 'PickupObject' skill, and the mass capacity each robot possesses. In this scenario, we have one main subtasks: 'Put Tomato in Fridge.'.
# For the 'Put Tomato in Fridge.' subtask, mass of the Tomato is 4. Hence the subtask can be performed by any robot with mass capacity greater than or equal to 4. In this case, Robots 1 has a mass capacity = 4.
# No teams are required since SubTasks can be performed with individual robots as explained above. The 'Put Tomato in Fridge.' subtask is assigned to Robot 1. 

# Code Solution 
def put_tomato_in_fridge(robot_list):
    # robot_list = [robot1]
    # 0: SubTask 1: Put Tomato in Fridge
    # 1: Go to the Tomato using robot1.
    GoToObject(robot_list,'Tomato')
    # 2: Pick up the Tomato using robot1.
    PickupObject(robot_list,'Tomato')
    # 3: Go to the Fridge using robot1.
    GoToObject(robot_list,'Fridge')
    # 4: Open the Fridge using robot1.
    OpenObject(robot_list,'Fridge')
    # 5: Put the Tomato in the Fridge using robot1.
    PutObject(robot_list,'Tomato', 'Fridge')
    # 6: Close the Fridge using robot1.
    CloseObject(robot_list,'Fridge')
# Perform SubTask 1 
put_tomato_in_fridge([robots[0]])

# Task Put tomato in fridge is done


# EXAMPLE 3 - Task Description: Slice the Potato 
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Slice the Potato. (Skills Required: GoToObject, PickupObject, SliceObject, PutObject)
# We can execute SubTask 1 first.

# CODE
def slice_potato():
    # 0: SubTask 1: Slice the Potato
    # 1: Go to the Knife.
    GoToObject('Knife')
    # 2: Pick up the Knife.
    PickupObject('Knife')
    # 3: Go to the Potato.
    GoToObject('Potato')
    # 4: Slice the Potato.
    SliceObject('Potato')
    # 5: Go to the countertop.
    GoToObject('CounterTop')
    # 6: Put the Knife back on the CounterTop.
    PutObject('Knife', 'CounterTop')
# Execute SubTask 1
slice_potato()
# Task fry sliced potato is done


# TASK ALLOCATION
robots = [{'name': 'robot1', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject'],'mass': 2}, {'name': 'robot2', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SwitchOn', 'SwitchOff', 'DropHandObject', 'ThrowObject', 'PushObject', 'PullObject'],'mass': 2}, {'name': 'robot3', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'PickupObject', 'PutObject', 'DropHandObject'],'mass': 2}]
# SOLUTION
# All the robots DONOT share the same set and number of skills (no_skills) & all objects have different masses. In this case where all robots have different sets of skills and objects have different mass - Focus on Task Allocation based on Robot Skills alone. 
# Analyze the skills required for each subtask and the skills each robot possesses. In this scenario, we have one main subtasks: 'Slice the Potato'.
# For the 'Slice the Potato' subtask, it can be performed by any robot with 'GoToObject', 'PickupObject', 'SliceObject' and 'PutObject' skills. However, no individual robot has all these skills. This is a skill gap that needs to be addressed. Form a team of robots. The skills of the team must be 'GoToObject', 'PickupObject', 'SliceObject' and 'PutObject' skills. Team of Robots 1 and 3 have all the skills required where robot 1 has the 'SliceObject' skill and Robot 3 has the 'GoToObject', 'PickupObject', and 'PutObject' skills.
# Teams are required since SubTasks can't be performed with individual robots as explained above. The 'Slice the Potato' subtask is assigned to team of Robots 1 and 3. 

# Code Solution
def slice_potato(robot_list):
    # robot_list = [robot1,robot3]
    # 0: SubTask 1: Slice the Potato
    # 1: Go to the Knife  using robot3.
    GoToObject(robot_list[1],'Knife')
    # 2: Pick up the Knife using robot3.
    PickupObject(robot_list[1],'Knife')
    # 3: Go to the Potato using robot3.
    GoToObject(robot_list[1],'Potato')
    # 4: Slice the Potato using robot1.
    SliceObject(robot_list[0],'Potato')
    # 5: Go to the countertop using robot3.
    GoToObject(robot_list[1],'CounterTop')
    # 6: Put the Knife back on the CounterTop using robot3.
    PutObject(robot_list[1],'Knife', 'CounterTop')
# Execute SubTask 1
slice_potato([robots[0],robots[2]])
# Task fry sliced potato is done


# EXAMPLE 4 - Task Description: Throw the fork in the trash
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Pick up the Fork. (Skills Required: GoToObject, PickupObject)
# SubTask 2: Throw the Fork in the Trash. (Skills Required: GoToObject, ThrowObject)
# We can execute SubTask 1 first and then SubTask 2.

# CODE
# CODE Solution
def pick_up_fork(robot_list):
    # robot_list = [robot1]
    r = robot_list[0]
    # 0: SubTask 1: Pick up the Fork
    GoToObject(r, 'Fork')
    PickupObject(r, 'Fork')

def throw_fork_in_trash(robot_list):
    # robot_list = [robot1]
    r = robot_list[0]
    # 0: SubTask 2: Throw the Fork in the Trash
    GoToObject(r, 'GarbageCan')
    ThrowObject(r, 'Fork', 'GarbageCan')

# Execute SubTask 1
pick_up_fork([robots[0]])
# Execute SubTask 2
throw_fork_in_trash([robots[0]])

# Task throw the fork in the trash is done


# TASK ALLOCATION
robots = [{'name': 'robot1', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject', 'PickupObject', 'PutObject', 'SwitchOn', 'SwitchOff', 'ThrowObject', 'PushObject', 'PullObject'],'mass': 3}, {'name': 'robot2', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject', 'PickupObject', 'PutObject', 'SwitchOn', 'SwitchOff', 'DropHandObject', 'ThrowObject', 'PushObject', 'PullObject'],'mass': 2}, {'name': 'robot3', 'skills': ['GoToObject', 'OpenObject', 'CloseObject', 'BreakObject', 'SliceObject', 'PickupObject', 'PutObject', 'SwitchOn', 'SwitchOff', 'DropHandObject', 'ThrowObject', 'PushObject', 'PullObject'],'mass': 2}]
# SOLUTION
# All the robots share the same set and number of skills (no_skills) & all objects DONOT have same mass. In this case where all objects have different mass, and robots have same sets of skills- Focus on Task Allocation based on Mass alone. 
# Analyze the mass required for each object being PickedUp by the 'PickupObject' skill, and the mass capacity each robot possesses. In this scenario, we have two main subtasks: 'Pick up the Fork' and 'Throw the Fork in the Trash'.
# For the 'Pick up the Fork' subtask, mass of the Fork is 5. Hence the subtask can be performed by any robot with mass capacity greater than or equal to 5. However, no individual robot has mass capacity of 5. This is a mass gap that needs to be addressed. Form a team of robots. The combined mass capacity of the team must be greater than or equal to 5. Team of Robots 1 and 2 have the mass capacity required where robot1 has mass capacity of 3 and where robot2 has mass capacity of 2 , this gives a combined mass capacity of 5.
# For the 'Throw the Fork in the Trash' subtask, mass of the Fork is 5. Hence the subtask can be performed by any robot with mass capacity greater than or equal to 5. However, no individual robot has mass capacity of 5. This is a mass gap that needs to be addressed. Form a team of robots. The combined mass capacity of the team must be greater than or equal to 5. Team of Robots 1 and 3 have the mass capacity required where robot1 has mass capacity of 3 and where robot3 has mass capacity of 2 , this gives a combined mass capacity of 5.
# Teams are required since SubTasks can't be performed with individual robots as explained above. The 'Pick up the Fork' subtask is assigned to team of Robots 1 and 2. The 'Throw the Fork in the Trash' subtask is assigned to team of Robots 1 and 3. 

# CODE Solution
def pick_up_fork(robot_list):
    # robot_list = [robot1,robot2]
    # 0: SubTask 1: Pick up the Fork
    # 1: Go to the Fork using robot1 and robot2 togethor.
    GoToObject(robot_list,'Fork')
    # 2: Pick up the Fork using robot1 and robot2 togethor.
    PickupObject(robot_list,'Fork')

def throw_fork_in_trash():
    # robot_list = [robot1,robot3]
    # 0: SubTask 2: Throw the Fork in the Trash
    # 1: Go to the GarbageCan using robot1 and robot3 togethor.
    GoToObject(robot_list,'GarbageCan')
    # 2: Throw the Fork in the GarbageCan using robot1 and robot3 togethor.
    ThrowObject(robot_list,'Fork', 'GarbageCan')

# Execute SubTask 1
pick_up_fork([robots[0],robots[1]])

# Execute SubTask 2
throw_fork_in_trash([robots[0],robots[2]])

# Task throw the fork in the trash is done