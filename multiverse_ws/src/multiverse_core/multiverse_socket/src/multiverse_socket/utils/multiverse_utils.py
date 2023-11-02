import rospy


def init_request_meta_data_dict(meta_data: dict) -> dict:
    request_meta_data_dict = {}
    request_meta_data_dict["world"] = meta_data.get("world", "world")
    request_meta_data_dict["length_unit"] = meta_data.get("length_unit", "m") 
    request_meta_data_dict["angle_unit"] = meta_data.get("angle_unit", "rad") 
    request_meta_data_dict["mass_unit"] = meta_data.get("mass_unit", "kg") 
    request_meta_data_dict["time_unit"] = meta_data.get("time_unit", "s") 
    request_meta_data_dict["handedness"] = meta_data.get("handedness", "rhs") 
    request_meta_data_dict["send"] = {}
    request_meta_data_dict["receive"] = {}

    return request_meta_data_dict
