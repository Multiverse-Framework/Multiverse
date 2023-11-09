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
        self.__send_data = []

    def update_world(self, world_name) -> None:
        super()._init_request_meta_data()
        self._request_meta_data_dict["meta_data"]["world_name"] = world_name
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
        print("request", request)
        meta_data = request.meta_data
        world_name = meta_data.world_name
        if world_name == "":
            import rospy
            rospy.logwarn(f"World is not set, set to default world")
            world_name = "world"
        
        simulation_name = meta_data.simulation_name
        if simulation_name == "":
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
        self._request_meta_data_dict["meta_data"] = {}
        self._request_meta_data_dict["meta_data"]["world_name"] = world_name
        self._request_meta_data_dict["meta_data"]["simulation_name"] = "ros" if simulation_name == "" else simulation_name
        self._request_meta_data_dict["meta_data"]["length_unit"] = "m" if meta_data.length_unit == "" else meta_data.length_unit
        self._request_meta_data_dict["meta_data"]["angle_unit"] = "rad" if meta_data.angle_unit == "" else meta_data.angle_unit
        self._request_meta_data_dict["meta_data"]["mass_unit"] = "kg" if meta_data.mass_unit == "" else meta_data.mass_unit
        self._request_meta_data_dict["meta_data"]["time_unit"] = "s" if meta_data.time_unit == "" else meta_data.time_unit
        self._request_meta_data_dict["meta_data"]["handedness"] = "rhs" if meta_data.handedness == "" else meta_data.handedness

        self._request_meta_data_dict["send"] = {}
        self._request_meta_data_dict["receive"] = {}

        self.__send_data = [0]

        empty_simulation_name = simulation_name == ""
        
        for object_data in request.send:
            empty_object_name = object_data.object_name == ""
            if not empty_simulation_name and empty_object_name:
                continue
            self._request_meta_data_dict["send"][object_data.object_name] = [object_data.attribute_name]
            self.__send_data += object_data.data

        for object_attribute in request.receive:
            empty_object_name = object_attribute.object_name == ""
            object_not_found = world_name not in self.__worlds or object_attribute.object_name not in self.__worlds[world_name]
            if empty_simulation_name and object_not_found:
                continue
            if not empty_simulation_name and empty_object_name:
                continue
            self._request_meta_data_dict["receive"][object_attribute.object_name] = []
            for attribute_name in object_attribute.attribute_names:
                empty_attribute_name = attribute_name == ""
                attribute_not_found = world_name not in self.__worlds or attribute_name not in self.__worlds[world_name][object_attribute.object_name]
                if empty_simulation_name and attribute_not_found:
                    continue
                if not empty_simulation_name and empty_attribute_name:
                    continue
                self._request_meta_data_dict["receive"][object_attribute.object_name].append(attribute_name)

    def _bind_response(self, response_meta_data_dict: Dict) -> SocketResponse:
        response = SocketResponse()
        response.meta_data.world_name = response_meta_data_dict["meta_data"]["world_name"]
        response.meta_data.simulation_name = response_meta_data_dict["meta_data"]["simulation_name"]
        response.meta_data.length_unit = response_meta_data_dict["meta_data"]["length_unit"]
        response.meta_data.angle_unit = response_meta_data_dict["meta_data"]["angle_unit"]
        response.meta_data.mass_unit = response_meta_data_dict["meta_data"]["mass_unit"]
        response.meta_data.time_unit = response_meta_data_dict["meta_data"]["time_unit"]
        response.meta_data.handedness = response_meta_data_dict["meta_data"]["handedness"]

        if len(self.__send_data) > 1:
            self._set_send_data(self.__send_data)
            self._communicate()
            self._communicate()
            self._request_meta_data_dict["name"] = "ros"
            self._set_request_meta_data()
            self._communicate(True)

            for object_name, object_data in self._get_response_meta_data()["send"].items():
                for attribute_name, attribute_data in object_data.items():
                    response.send.append(ObjectData(object_name, attribute_name, attribute_data))
                    
        if response_meta_data_dict.get("receive") is None:
            return response

        for object_name, object_data in response_meta_data_dict["receive"].items():
            for attribute_name, attribute_data in object_data.items():
                response.receive.append(ObjectData(object_name, attribute_name, attribute_data))

        return response
