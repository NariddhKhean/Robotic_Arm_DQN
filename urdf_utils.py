import pybullet as p
import math

def find_joint_count(ID):
    """Counts the number of joints in robot.

    Arguments:
        ID -- An integer representing a robot, as return from the p.loadURDF()
        method.

    Returns:
        joint_count -- An integer representing the number of joints as defined
        in the *.urdf file.
    """

    joint_count = 0
    while True:
        try:
            p.getJointState(ID, joint_count)
            joint_count += 1
        except:
            return joint_count

def test_revolute_extents(ID, joint_count, speed=0.2, precision=5, delay=0.001):
    """Iterates through each joint and rotates them to the extent of its
    movement. Returns a list of tuples representing the (minimum, maximum) of
    each joint. Note: this has only been tested on robots with only revolute
    joints.

    Arguments:
        ID          -- An integer representing a robot, as return from the
        p.loadURDF() method.
        joint_count -- An integer representing the number of joints as defined
        in the *.urdf file. Can be calculated with the find_joint_count()
        function.
        speed       -- A number representing the rate at which each link rotates.
        precision   -- An integer representing the number of decimal places to
        which the extents will be calculated to.
        delay       -- A number that artificially delays the simulation steps
        with time.sleep(delay).

    Returns:
        extents -- A list of tuples representing the minimum and maximum extents
        of rotation for each joint.
    """

    # Set Starting States
    starting_states = [p.getJointState(ID, i)[0] for i in range(joint_count)]

    # Extents
    extents = [2 * [starting_states[i]] for i in range(joint_count)]

    # Initial Search Space
    current_joint = 0
    current_direction = -1

    # Search
    while True:

        # Check for Final Joint
        if current_joint == joint_count:
            extent = [(180 * rad / math.pi for rad in pair) for pair in extents]
            return [tuple(pair) for pair in extent]

        # Update Extents
        current_state = p.getJointState(ID, current_joint)[0]
        current_min = extents[current_joint][0]
        current_max = extents[current_joint][1]
        if current_state < current_min:
            extents[current_joint][0] = current_state
        if current_state > current_max:
            extents[current_joint][1] = current_state

        # Make Move
        target_state = current_state + (current_direction * speed)
        p.setJointMotorControl2(
            bodyIndex=ID,
            jointIndex=current_joint,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_state,
        )
        p.stepSimulation()
        if delay > 0:
            time.sleep(delay)

        # Checks if at Extent
        new_state = p.getJointState(ID, current_joint)[0]
        new_state_string = str(int(new_state * (10**precision)))
        cur_state_string = str(int(current_state * (10**precision)))
        if new_state_string == cur_state_string:
            if current_direction == 1:
                if current_joint < joint_count:
                    # Reset Joint
                    p.setJointMotorControl2(
                        bodyIndex=ID,
                        jointIndex=current_joint,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=starting_states[current_joint],
                    )
                    current_joint += 1
            current_direction *= -1