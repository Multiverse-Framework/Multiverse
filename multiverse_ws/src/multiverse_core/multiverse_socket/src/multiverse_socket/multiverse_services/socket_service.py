#!/usr/bin/env python3

from typing import Dict
from multiverse_socket.multiverse_services import MultiverseRosServiceServer
from multiverse_msgs.msg import ObjectAttribute, ObjectData
from multiverse_msgs.srv import Socket, SocketRequest, SocketResponse


class socket_service(MultiverseRosServiceServer):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._service_name = "/multiverse/socket"
        self._service_class = Socket
        self.__worlds = {}

    def update_world(self, world_name) -> None:
        super()._init_request_meta_data()
        self._request_meta_data_dict["world"] = ""
        self._request_meta_data_dict["receive"][""] = [""]
        self._set_request_meta_data()
        self._communicate(True)
        response_meta_data_dict = self._get_response_meta_data()
        if response_meta_data_dict:
            self.__worlds[world_name] = {}
            self.__worlds[world_name][""] = {""}
            if "receive" not in response_meta_data_dict:
                return
            for object_name, object_data in response_meta_data_dict["receive"].items():
                self.__worlds[world_name][object_name] = {""}
                for attribute_name in object_data:
                    self.__worlds[world_name][""].add(attribute_name)
                    self.__worlds[world_name][object_name].add(attribute_name)

    def _bind_request_meta_data(self, request: SocketRequest) -> None:
        world_name = request.world
        world_need_update = False

        if self.__worlds.get(world_name) is None:
            world_need_update = True
        else:
            object_attribute: ObjectAttribute
            for object_attribute in request.receive:
                if self.__worlds[world_name].get(object_attribute.object_name) is None:
                    world_need_update = True
                    break
                for attribute_name in object_attribute.attribute_names:
                    if attribute_name not in self.__worlds[world_name][object_attribute.object_name]:
                        world_need_update = True
                        break

        if world_need_update:
            self.update_world(world_name)

        self._request_meta_data_dict = {}
        self._request_meta_data_dict["world"] = world_name
        self._request_meta_data_dict["length_unit"] = "m" if request.length_unit == "" else request.length_unit
        self._request_meta_data_dict["angle_unit"] = "rad" if request.angle_unit == "" else request.angle_unit
        self._request_meta_data_dict["mass_unit"] = "kg" if request.mass_unit == "" else request.mass_unit
        self._request_meta_data_dict["time_unit"] = "s" if request.time_unit == "" else request.time_unit
        self._request_meta_data_dict["handedness"] = "rhs" if request.handedness == "" else request.handedness

        self._request_meta_data_dict["send"] = {}
        self._request_meta_data_dict["receive"] = {}

        for object_attribute in request.receive:
            if world_name not in self.__worlds or object_attribute.object_name not in self.__worlds[world_name]:
                continue
            self._request_meta_data_dict["receive"][object_attribute.object_name] = []
            for attribute_name in object_attribute.attribute_names:
                if attribute_name not in self.__worlds[world_name][object_attribute.object_name]:
                    continue
                self._request_meta_data_dict["receive"][object_attribute.object_name].append(attribute_name)

    def _bind_response(self, response_meta_data_dict: Dict) -> SocketResponse:
        response = SocketResponse()
        response.world = response_meta_data_dict["world"]
        response.length_unit = response_meta_data_dict["length_unit"]
        response.angle_unit = response_meta_data_dict["angle_unit"]
        response.mass_unit = response_meta_data_dict["mass_unit"]
        response.time_unit = response_meta_data_dict["time_unit"]
        response.handedness = response_meta_data_dict["handedness"]

        if response_meta_data_dict.get("receive") is None:
            return response

        for object_name, object_data in response_meta_data_dict["receive"].items():
            for attribute_name, attribute_data in object_data.items():
                response.receive.append(ObjectData(object_name, attribute_name, attribute_data))

        return response
