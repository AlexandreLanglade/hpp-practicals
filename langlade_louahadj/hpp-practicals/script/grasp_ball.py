from manipulation import robot, vf, ps, Ground, Box, Pokeball, PathPlayer, gripperName, ballName
from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client
Client().problem.resetProblem()

vf.loadEnvironmentModel(Ground, 'ground')
vf.loadObjectModel(Pokeball, 'pokeball')
robot.setJointBounds('pokeball/root_joint', [-.4, .4, -.4, .4, -.1, 2.,
                                             -1.0001, 1.0001, -1.0001, 1.0001,
                                             -1.0001, 1.0001, -1.0001, 1.0001, ])
q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]
graph = ConstraintGraph(robot, 'graph')
ballInGripper = [0, .137, 0, 0.5, 0.5, -0.5, 0.5]
ps.createTransformationConstraint('grasp', gripperName, ballName,
                                  ballInGripper, 6*[True, ])
gripperAboveBall = [0., 0.237, 0., 0.5, 0.5, -0.5, 0.5]
ps.createTransformationConstraint('gripper-above-ball', gripperName,
                                  ballName, gripperAboveBall, [True, True, True, True, True, True, ])
graph.createNode(['grasp', 'placement', 'gripper-above-ball'])
graph.createEdge('placement', 'gripper-above-ball',
                 'approach-ball', 1, 'placement')
graph.createEdge('gripper-above-ball', 'gripper-above-ball',
                 'above', 1, 'gripper-above-ball')
graph.createEdge('placement', 'placement', 'transit', 1, 'placement')
graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')
graph.createEdge('placement', 'grasp', 'grasp-ball', 1, 'placement')
graph.createEdge('grasp', 'placement', 'release-ball', 1, 'grasp')
ps.createTransformationConstraint('placement', '', ballName,
                                  [0, 0, 0.025, 0, 0, 0, 1],
                                  [False, False, True, True, True, False, ])
ps.createTransformationConstraint('placement/complement', '', ballName,
                                  [0, 0, 0.025, 0, 0, 0, 1],
                                  [True, True, False, False, False, True, ])
ps.setConstantRightHandSide('placement', True)
ps.setConstantRightHandSide('placement/complement', False)
graph.addConstraints(node='gripper-above-ball',
                     constraints=Constraints(numConstraints=['placement']))
graph.addConstraints(node='gripper-above-ball',
                     constraints=Constraints(numConstraints=['gripper-above-ball']))
graph.addConstraints(node='placement', constraints=Constraints(
    numConstraints=['placement'],))
graph.addConstraints(node='grasp',
                     constraints=Constraints(numConstraints=['grasp']))
graph.addConstraints(edge='approach-ball',
                     constraints=Constraints(numConstraints=['placement/complement']))
graph.addConstraints(edge='transit', constraints=Constraints(
    numConstraints=['placement/complement']))
graph.addConstraints(
    edge='grasp-ball', constraints=Constraints(numConstraints=['placement/complement']))
graph.addConstraints(edge='transfer',     constraints=Constraints())
graph.addConstraints(edge='release-ball', constraints=Constraints())
ps.selectPathValidation("Dichotomy", 0)
ps.selectPathProjector("Progressive", 0.1)
graph.initialize()
res, q_init, error = graph.applyNodeConstraints('placement', q1)
q2 = q1[::]
q2[7] = .2
res, q_goal, error = graph.applyNodeConstraints('placement', q2)
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
for i in range(100):
    q = robot.shootRandomConfig()
    res, q3, err = graph.generateTargetConfig('grasp-ball', q_init, q)
    if res and robot.isConfigValid(q3):
        break
if res:
    robot.setCurrentConfig(q3)
    gripperPose = Transform(robot.getJointPosition(gripperName))
    ballPose = Transform(robot.getJointPosition(ballName))
    gripperGraspsBall = gripperPose.inverse() * ballPose
    gripperAboveBall = Transform(gripperGraspsBall)
    gripperAboveBall.translation[2] += .1
for i in range(100):
    qrand = robot.shootRandomConfig()
    b, qproj, _ = graph.applyNodeConstraints('placement', qrand)
    if (b):
        break
for i in range(100):
    qrand = robot.shootRandomConfig()
    b, qtrans, _ = graph.generateTargetConfig('approach-ball', qproj, qrand)
    if b:
        break
