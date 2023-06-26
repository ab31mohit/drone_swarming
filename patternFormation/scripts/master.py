import rospy
from geometry_msgs.msg import Vector3, Pose
from gazebo_msgs.msg import ModelStates
import json
import os


rospy.init_node('base', anonymous=True)
model_poses = [0 for i in range(12)] #pose array of type geometry_msgs/Pose
init_pose = [0 for i in range(12)]   #initial position of drones
one_time = True
def _set_odom(modelstates):
	global one_time
	for i in range(12):
		drone_id = f"drone{i+1}"
		index = modelstates.name.index(drone_id)
		posn = modelstates.pose[index].position
		if one_time:
			init_pose[i] = posn
			one_time = False
		model_poses[i] = posn


rospy.Subscriber("/gazebo/model_states", ModelStates, _set_odom)
pub_arr = [rospy.Publisher(f"D{i+1}", Vector3, queue_size=10) for i in range(12)]

path = "/home/mohit/workspace/drone_ws/src/drone_swarming/patternFormation/pattern_data_output"
file_name = input("Enter the json file name (with extension) for Pattern formation:\n") 
file_path = os.path.join(path, file_name)

f = open(file_path)
data = json.load(f)

if not rospy.is_shutdown():
	v=Vector3()
	X=[data['drone'+str(i+1)]['x'] for i in range(12)]
	Y=[data['drone'+str(i+1)]['y'] for i in range(12)]
	Z=[data['drone'+str(i+1)]['z'] for i in range(12)]
	# send the final goals
	rate = rospy.Rate(10)
	while True:
		for i in range(12):
			v.x=X[i]
			v.y=Y[i]
			v.z=Z[i]
			pub_arr[i].publish(v)
		rate.sleep()
		print(f"Publishing {file_name} data as goal Position for each drone!\n")
rospy.spin()
