import rospy


def init_request_meta_data_dict(world: str) -> dict:
    request_meta_data_dict = {}
    request_meta_data_dict["world"] = world
    request_meta_data_dict["length_unit"] = rospy.get_param("/multiverse_client/ros/length_unit") if rospy.has_param("/multiverse_client/ros/length_unit") else "m"
    request_meta_data_dict["angle_unit"] = rospy.get_param("/multiverse_client/ros/angle_unit") if rospy.has_param("/multiverse_client/ros/angle_unit") else "rad"
    request_meta_data_dict["force_unit"] = rospy.get_param("/multiverse_client/ros/force_unit") if rospy.has_param("/multiverse_client/ros/force_unit") else "N"
    request_meta_data_dict["time_unit"] = rospy.get_param("/multiverse_client/ros/time_unit") if rospy.has_param("/multiverse_client/ros/time_unit") else "s"
    request_meta_data_dict["handedness"] = rospy.get_param("/multiverse_client/ros/handedness") if rospy.has_param("/multiverse_client/ros/handedness") else "rhs"
    request_meta_data_dict["send"] = {}
    request_meta_data_dict["receive"] = {}

    return request_meta_data_dict
