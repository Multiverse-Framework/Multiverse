def init_request_meta_data_dict(meta_data: dict) -> dict:
    request_meta_data_dict = {}
    request_meta_data_dict["meta_data"] = {}
    request_meta_data_dict["meta_data"]["world_name"] = meta_data.get("world_name", "world")
    request_meta_data_dict["meta_data"]["simulation_name"] = meta_data.get("simulation_name", "ros")
    request_meta_data_dict["meta_data"]["length_unit"] = meta_data.get("length_unit", "m") 
    request_meta_data_dict["meta_data"]["angle_unit"] = meta_data.get("angle_unit", "rad") 
    request_meta_data_dict["meta_data"]["mass_unit"] = meta_data.get("mass_unit", "kg") 
    request_meta_data_dict["meta_data"]["time_unit"] = meta_data.get("time_unit", "s") 
    request_meta_data_dict["meta_data"]["handedness"] = meta_data.get("handedness", "rhs") 
    request_meta_data_dict["send"] = {}
    request_meta_data_dict["receive"] = {}

    return request_meta_data_dict
