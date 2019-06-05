import tensorflow as tf
import pybullet as p
import pybullet_data
import numpy as np
import datetime
import random
import math
import json
import os

import dirs


def import_config(config_path):
    with open(config_path) as f:
        config = json.load(f)
    return config

def initialise_env():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
    p.setGravity(0, 0, -9.8)

    kuka = p.loadURDF('kuka_iiwa/model.urdf', [0, 0, 0], useFixedBase=True)

    return kuka

def kuka_camera(cam_dim, eff_coords, eff_quatrn, init_cam_vec, init_up_vec):
    q_matrix      = p.getMatrixFromQuaternion(eff_quatrn)
    rot_matrix    = np.array(q_matrix).reshape(3, 3)
    camera_vector = rot_matrix.dot(init_cam_vec)
    up_vector     = rot_matrix.dot(init_up_vec)

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60,
        aspect=1.0,
        nearVal=0.01,
        farVal=100
    )
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=eff_coords,
        cameraTargetPosition=eff_coords + 0.1 * camera_vector,
        cameraUpVector=up_vector,
    )
    image = p.getCameraImage(
        width=cam_dim,
        height=cam_dim,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
    )

    rgba = image[2]
    rgb  = []
    for row in range(len(rgba)):
        rgb.append([])
        for col in range(len(rgba[row])):
            rgb[row].append(rgba[row][col][:3].tolist())

    return np.array(rgb) / 255

def build_conv_layer(input_tensor, filters, kernel_size):
    x = tf.keras.layers.Conv2D(filters, kernel_size)(input_tensor)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.LeakyReLU(0.01)(x)
    x = tf.keras.layers.MaxPool2D(2, 2)(x)
    return x

def build_dense_layer(input_tensor, neurons):
    x = tf.keras.layers.Dense(neurons)(input_tensor)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.LeakyReLU(0.01)(x)
    return x

def build_model(input_image_dim, input_state_dim, output_dim):

    # Camera Input
    image_input = tf.keras.layers.Input(
        shape=(input_image_dim, input_image_dim, 3),
        name='image_input'
    )
    x = build_conv_layer(input_tensor=image_input, filters=32, kernel_size=3)
    x = build_conv_layer(input_tensor=x, filters=64, kernel_size=3)
    x = build_conv_layer(input_tensor=x, filters=128, kernel_size=3)
    x = tf.keras.layers.Flatten()(x)

    # Kuka Joint Input
    joint_input = tf.keras.layers.Input(
        shape=(input_state_dim,),
        name='joint_input'
    )
    y = build_dense_layer(input_tensor=joint_input, neurons=1024)
    y = build_dense_layer(input_tensor=y, neurons=512)
    y = build_dense_layer(input_tensor=y, neurons=256)

    # Concatenate Two Inputs
    concat = tf.keras.layers.concatenate([x, y])

    # Output
    z = build_dense_layer(input_tensor=concat, neurons=1024)
    z = build_dense_layer(input_tensor=z, neurons=512)
    z = build_dense_layer(input_tensor=z, neurons=256)
    output = tf.keras.layers.Dense(output_dim, activation='softmax')(x)

    # Compile Model
    model = tf.keras.Model([image_input, joint_input], output)
    model.compile(loss='categorical_crossentropy', optimizer='adam')
    print(model.summary())

    return model

def save_model(model, base_name, iteration):
    base_name = base_name.replace(' ', '.')
    base_name = base_name.replace(':', '.')
    base_name = base_name.replace('-', '.')

    model_dir = os.path.join(dirs.MODELS_DIR, base_name)
    if not os.path.isdir(model_dir):
        os.makedirs(model_dir)

    model_name = os.path.join(model_dir, '{}.h5'.format(iteration))
    model.save(model_name)

def e_greedy_policy(q_estimates, output_neurons):
    if random.random() <= epsilon:
        return int(random.randint(0, output_neurons - 1)), True
    else:
        return int(np.argmax(q_estimates)), False

def train_dqn(config):
    global epsilon
    model = build_model(
        input_image_dim=config['kuka_cam_dim'],
        input_state_dim=config['kuka_axis_count'],
        output_dim=config['kuka_action_dim']
    )
    kuka  = initialise_env()

    # Initialise Memory and Epsilon
    memory  = []
    epsilon = config['init_epsilon']

    # Set Up Visualisation
    p.resetDebugVisualizerCamera(
        cameraTargetPosition=config['viewport_pos'],
        cameraDistance=config['viewport_dist'],
        cameraPitch=config['viewport_pitch'],
        cameraYaw=config['viewport_yaw']
    )

    prev_state = []

    # Set Up Saving Name
    model_name = str(datetime.datetime.now())

    # Training Loop
    i = 0
    while True:

        # Reset Episode
        if i % config['simulation_reset_iter'] == 0:
            print('\n-- RESET EPISODE --')
            for j in range(config['kuka_axis_count']):
                p.resetJointState(
                    bodyUniqueId=kuka,
                    jointIndex=j,
                    targetValue=config['kuka_reset_pose'][j]
                )

            obj_starting_pos = [-0.8, 0., 0.02]
            if i == 0:
                obj = p.loadURDF('block.urdf', obj_starting_pos)
            p.resetBasePositionAndOrientation(
                bodyUniqueId=obj,
                posObj=obj_starting_pos,
                ornObj=[0, 0, 0, 1]
            )

        # Step Simulation
        steps = config['simulation_steps_per_action']
        [p.stepSimulation() for _ in range(steps)]

        # Set Up Camera
        effector_state = p.getLinkState(
            bodyUniqueId=kuka,
            linkIndex=config['kuka_axis_count'] - 1,
            computeForwardKinematics=True
        )

        # Calculate Input States
        image_state = kuka_camera(
            cam_dim=config['kuka_cam_dim'],
            eff_coords=effector_state[0],
            eff_quatrn=effector_state[1],
            init_cam_vec=(0, 0, 1),
            init_up_vec=(1, 0, 0)
        )
        axes_range = list(range(config['kuka_axis_count']))
        joint_state = [p.getJointState(kuka, j)[0] for j in axes_range]
        input_state = {
            'image_input': np.array([image_state]),
            'joint_input': np.array([joint_state])
        }

        # Select Action
        q_estimates = model.predict_on_batch(input_state)[0]
        action, random_act = e_greedy_policy(
            q_estimates=q_estimates,
            output_neurons=config['kuka_action_dim']
        )

        # Decay Epsilon
        epsilon = config['final_epsilon'] + (1 - config['lambda']) * \
            (epsilon - config['final_epsilon'])

        # Target Joint States
        target_joint_state = joint_state.copy()
        joint_index        = action//2
        if action % 2 == 0:
            target_joint_state[joint_index] += config['kuka_rotation_rad']
        else:
            target_joint_state[joint_index] -= config['kuka_rotation_rad']
        for j, joint in enumerate(target_joint_state):
            if joint > config['kuka_joint_range'][j] / 2:
                target_joint_state[j] = config['kuka_joint_range'][j]
            elif joint < -config['kuka_joint_range'][j] / 2:
                target_joint_state[j] = -config['kuka_joint_range'][j]

        # Control
        p.setJointMotorControlArray(
            bodyIndex=kuka,
            jointIndices=range(config['kuka_axis_count']),
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_joint_state,
            targetVelocities=[0] * config['kuka_axis_count'],
            forces=[500] * config['kuka_axis_count'],
            positionGains=[0.03] * config['kuka_axis_count'],
            velocityGains=[1] * config['kuka_axis_count']
        )

        # Playing Around with Reward Schema
        obj_pos  = p.getBasePositionAndOrientation(obj)[0]
        # obj_dist = math.sqrt(obj_pos[0]**2 + obj_pos[1]**2)
        # obj_dist = math.sqrt((obj_pos[0]-prev_xy[0])**2 + \
        #            (obj_pos[1]-prev_xy[1])**2)
        # reward = obj_dist
        # reward = (effector_state[0][2] - 0.5) * 10
        # prev_xy = [obj_pos[0], obj_pos[1]]
        reward = math.sqrt(
            (effector_state[0][0] - obj_pos[0])**2 + \
            (effector_state[0][1] - obj_pos[1])**2 + \
            (effector_state[0][2] - obj_pos[2])**2) * -1 + 1

        if i % config['simulation_reset_iter'] != 0:
            print('\n  ITERATION: {}'.format(i))
            print('  joint_state = {}'.format(joint_state))
            print('  q-estimates = {}'.format(q_estimates.tolist()))
            print('  action      = {}'.format(action), end='')
            if random_act == True:
                print(' (epsilon)', end='')
            print('\n  reward      = {}'.format(reward))
            print('  epsilon     = {}'.format(epsilon))

            # Store Memory Sample
            current_state = [image_state, joint_state]
            memory_sample = [prev_state, action, reward, current_state]
            memory.append(memory_sample)

            # Pop Excess Memory
            if len(memory) > config['max_memeory']:
                memory.pop(0)

            # Sample Batch from Memory
            mini_batch_size = min(config['batch_size'], len(memory))
            batch = random.sample(memory, mini_batch_size)

            # Training Input and Target Arrays for Batch
            states = {
                'image_input': np.array([sample[0][0] for sample in batch]),
                'joint_input': np.array([sample[0][1] for sample in batch])
            }
            next_states = {
                'image_input': np.array([sample[3][0] for sample in batch]),
                'joint_input': np.array([sample[3][1] for sample in batch])
            }

            # Predict Q-Values for Batch
            q_s_a   = model.predict_on_batch(states)
            q_s_a_d = model.predict_on_batch(next_states)

            # Set up Arrays for Training
            image_x = np.zeros(shape=(mini_batch_size,
                               config['kuka_cam_dim'],
                               config['kuka_cam_dim'],
                               3))
            joint_x = np.zeros(shape=(mini_batch_size,
                               config['kuka_axis_count']))
            targets = np.zeros(shape=(mini_batch_size,
                               config['kuka_action_dim']))

            # Update Training Data
            for index, sample in enumerate(batch):

                image_x[index] = sample[0][0]
                joint_x[index] = sample[0][1]

                q = q_s_a[index]

                q[sample[1]] = config['alpha'] * (sample[2] + \
                    config['gamma'] * np.amax(q_s_a_d[index]))
                targets[index] = q

            # Train Model
            training_inputs = {
                'image_input': np.array(image_x),
                'joint_input': np.array(joint_x)
            }
            model.train_on_batch(training_inputs, targets)

        # Update Previous State
        prev_state = [image_state, joint_state]

        if i % config['model_save_interval'] == 0:
            save_model(model=model, base_name=model_name, iteration=i)

        # Update Counter
        i += 1


if __name__ == '__main__':
    config = import_config(dirs.CONFIG_PATH)
    train_dqn(config)