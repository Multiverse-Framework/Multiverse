#!/usr/bin/env python3

from typing import Dict
from multiverse_socket.multiverse_services import MultiverseRosServiceServer
from multiverse_msgs.srv import SpawnObjects, SpawnObjectsRequest, SpawnObjectsResponse

class spawn_objects_service(MultiverseRosServiceServer):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._service_name = "/multiverse/spawn_objects"
        self._service_class = SpawnObjects
        self.__send_data = []

    def _bind_request_meta_data(self, request: SpawnObjectsRequest) -> None:
        self._request_meta_data_dict = {}
        self._request_meta_data_dict["name"] = request.simulation_name
        self._request_meta_data_dict["length_unit"] = "m" if request.length_unit == "" else request.length_unit
        self._request_meta_data_dict["angle_unit"] = "rad" if request.angle_unit == "" else request.angle_unit
        self._request_meta_data_dict["mass_unit"] = "kg" if request.mass_unit == "" else request.mass_unit
        self._request_meta_data_dict["time_unit"] = "s" if request.time_unit == "" else request.time_unit
        self._request_meta_data_dict["handedness"] = "rhs" if request.handedness == "" else request.handedness
        self._request_meta_data_dict["send"] = {}
        self._request_meta_data_dict["receive"] = {}

        self.__send_data = [0.0]
        for object_data in request.send:
            if object_data.object_name == "":
                continue
            self._request_meta_data_dict["send"][object_data.object_name] = [object_data.attribute_name]
            self.__send_data += object_data.data

        for object_attribute in request.receive:
            if object_attribute.object_name == "":
                continue
            self._request_meta_data_dict["receive"][object_attribute.object_name] = []
            for attribute_name in object_attribute.attribute_names:
                self._request_meta_data_dict["receive"][object_attribute.object_name].append(attribute_name)

    def _bind_response(self, response_meta_data_dict: Dict) -> SpawnObjectsResponse:
        response = SpawnObjectsResponse()

        if len(self.__send_data) > 1:
            self._set_send_data(self.__send_data)
            self._communicate()
            self._communicate()
            self._request_meta_data_dict["world"] = response_meta_data_dict["world"]
            self._request_meta_data_dict["name"] = "ros"
            self._set_request_meta_data()
            self._communicate(True)

        if response_meta_data_dict.get("send") is not None:
            response.send_objects = list(response_meta_data_dict["send"].keys())

        if response_meta_data_dict.get("receive") is not None:
            response.receive_objects = list(response_meta_data_dict["receive"].keys())

        return response