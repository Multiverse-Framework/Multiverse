import rospy


def init_request_meta_data_dict() -> dict:
    request_meta_data_dict = {}
    request_meta_data_dict["world"] = rospy.get_param("multiverse/world") if rospy.has_param("multiverse/world") else "world"
    request_meta_data_dict["length_unit"] = rospy.get_param("multiverse/length_unit") if rospy.has_param("multiverse/length_unit") else "m"
    request_meta_data_dict["angle_unit"] = rospy.get_param("multiverse/angle_unit") if rospy.has_param("multiverse/angle_unit") else "rad"
    request_meta_data_dict["force_unit"] = rospy.get_param("multiverse/force_unit") if rospy.has_param("multiverse/force_unit") else "N"
    request_meta_data_dict["time_unit"] = rospy.get_param("multiverse/time_unit") if rospy.has_param("multiverse/time_unit") else "s"
    request_meta_data_dict["handedness"] = rospy.get_param("multiverse/handedness") if rospy.has_param("multiverse/handedness") else "rhs"
    request_meta_data_dict["send"] = {}
    request_meta_data_dict["receive"] = {}

    return request_meta_data_dict
