import openravepy
import numpy as np

class AddConstraints():


	def make_collision_box_body(self,kinbody, add_to_pos=np.array([0.0, 0.0, 0.0]), add_to_extents = np.array([0.0, 0.0, 0.0])):
	  """Creates a box at position of kin body with optionally modified values
	  @param kinbody body we want to create a box around
	  @param env openrave environment
	  @param add_to_pos modifications to the position of the axis aligned bounding box
	  @param add_to_extents modifications to the extents of the axis aligned bounding box
	  @return kinbody of box object
	  """
	  env = kinbody.GetEnv()

	  with env:
	    old_transform_body = kinbody.GetTransform()
	    kinbody.SetTransform(np.eye(4))

	    
	    kinbody_aabb = kinbody.ComputeAABB()
	    box_aabb_arr = np.append(kinbody_aabb.pos()+add_to_pos, kinbody_aabb.extents()+add_to_extents)
	    box_aabb_arr = box_aabb_arr.reshape(1,6)
	    #print 'aabb box: ' + str(box_aabb_arr)

	    box_body = openravepy.RaveCreateKinBody(env, '')
	    box_body.SetName(kinbody.GetName() + '_box')
	    box_body.InitFromBoxes(box_aabb_arr, False)

	    env.Add(box_body)

	    kinbody.SetTransform(old_transform_body)
	    box_body.SetTransform(old_transform_body)
	    
	    box_body.Enable(True)

	  return box_body

	def AddConstraintBoxes(self,env, robot, handedness='right', name_base="constraint_boxes_", visible=False):
	  """Modifies environment to keep the robot inside a defined space
	    Does so by adding invisible boxes around the robot, which planners
	    avoid collisions with

	    @param env: openrave environment object
	    @param robot: openrave environment object
	    @param handedness: right or left
            @param name_base: string for boxes name
            @param visible: visibility of object
	    """


	    #add a box behind the robot
	    box_behind = openravepy.RaveCreateKinBody(env,'')
	    box_behind.SetName(name_base + 'behind')
	    box_behind.InitFromBoxes(np.array([[0.,0.,0., 0.4, 0.1, 1.0]]), False)
	    env.Add(box_behind)
	    T = np.eye(4)
	    T[0:3,3] = robot.GetTransform()[0:3,3]
	    T[1,3] = 0.57
	    if handedness == 'right':
	        T[0,3] += 0.25
	    else:
	        T[0,3] -= 0.25
	    box_behind.SetTransform(T)


	    #add a box above so we don't swing that way too high
	    box_above = openravepy.RaveCreateKinBody(env,'')
	    box_above.SetName(name_base + 'above')
	    box_above.InitFromBoxes(np.array([[0.,0.,0., 0.5, 0.5, 0.1]]), visible)
	    env.Add(box_above)
	    T = np.eye(4)
	    T[0:3,3] = robot.GetTransform()[0:3,3]
	    T[0,3] += 0.25
	    T[1,3] -= 0.25
	    T[2,3] += 0.90
	    box_above.SetTransform(T)


	    box_left = openravepy.RaveCreateKinBody(env,'')
	    box_left.SetName(name_base + 'left')
	    box_left.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
	    env.Add(box_left)
	    T = np.eye(4)
	    T[0:3,3] = robot.GetTransform()[0:3,3]
	    if handedness == 'right':
	        T[0,3] += 0.9
	    else:
	        T[0,3] += 0.25
	    T[1,3] = 0.25
	    box_left.SetTransform(T)

	    box_right = openravepy.RaveCreateKinBody(env,'')
	    box_right.SetName(name_base + 'right')
	    box_right.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
	    env.Add(box_right)
	    T = np.eye(4)
	    T[0:3,3] = robot.GetTransform()[0:3,3]
	    if handedness == 'right':
	        T[0,3] -= 0.25
	    else:
	        T[0,3] -= 0.9
	    T[1,3] = 0.25
	    box_right.SetTransform(T)
    

