import rospy

def constuct_send_meta_data_dict() -> dict:
    send_meta_data_dict = {}
    send_meta_data_dict["world"] = rospy.get_param(
        'multiverse/world') if rospy.has_param('multiverse/world') else "world"
    send_meta_data_dict["length_unit"] = rospy.get_param(
        "multiverse/length_unit") if rospy.has_param("multiverse/length_unit") else "m"
    send_meta_data_dict["angle_unit"] = rospy.get_param(
        "multiverse/angle_unit") if rospy.has_param("multiverse/angle_unit") else "rad"
    send_meta_data_dict["force_unit"] = rospy.get_param(
        "multiverse/force_unit") if rospy.has_param("multiverse/force_unit") else "N"
    send_meta_data_dict["time_unit"] = rospy.get_param(
        "multiverse/time_unit") if rospy.has_param("multiverse/time_unit") else "s"
    send_meta_data_dict["handedness"] = rospy.get_param(
        "multiverse/handedness") if rospy.has_param("multiverse/handedness") else "rhs"
    send_meta_data_dict["send"] = {}
    send_meta_data_dict["receive"] = {}

    return send_meta_data_dict