#!/usr/bin/env python3

from typing_extensions import Dict

from .multiverse_publishers.multiverse_publisher import MultiversePublisher
from .multiverse_services.multiverse_service import MultiverseService
from .multiverse_subscribers.multiverse_subscriber import MultiverseSubscriber
from .multiverse_node import MultiverseMetaData


class MultiverseNodeProperties:
    ros_node_name: str
    ros_node_prop: Dict
    multiverse_meta_data: MultiverseMetaData = MultiverseMetaData()
    client_port: str

    def __init__(self, ros_node_name: str, ros_name_prop: Dict):
        self.ros_node_name = ros_node_name
        self.ros_node_prop = ros_name_prop
        if "meta_data" in self.ros_node_prop:
            meta_data = self.ros_node_prop.pop("meta_data")
            self.multiverse_meta_data = MultiverseMetaData(
                world_name=meta_data.get("world_name", "world"),
                length_unit=meta_data.get("length_unit", "m"),
                angle_unit=meta_data.get("angle_unit", "rad"),
                mass_unit=meta_data.get("mass_unit", "kg"),
                time_unit=meta_data.get("time_unit", "s"),
                handedness=meta_data.get("handedness", "rhs"),
            )
        self.port = str(self.ros_node_prop.pop("port"))

    @property
    def ros_node_name(self):
        return self._ros_node_name

    @ros_node_name.setter
    def ros_node_name(self, ros_node_name: str):
        ros_node_name = ros_node_name.split("_")
        ros_node_name = "".join(part.capitalize() for part in ros_node_name)
        self._ros_node_name = ros_node_name

    def create_publisher(self) -> MultiversePublisher:
        topic_name = self.ros_node_prop.pop("topic") if "topic" in self.ros_node_prop else None
        rate = self.ros_node_prop.pop("rate")
        publisher_name = f"{self.ros_node_name}Publisher"
        for subclass in MultiversePublisher.__subclasses__():
            if subclass.__name__ == publisher_name:
                return subclass(
                    topic_name=topic_name,
                    rate=rate,
                    port=self.port,
                    multiverse_meta_data=self.multiverse_meta_data,
                    **self.ros_node_prop,
                )

        raise TypeError(f"Class {publisher_name} not found.")

    def create_subscriber(self) -> MultiverseSubscriber:
        topic_name = self.ros_node_prop.pop("topic")
        subscriber_name = f"{self.ros_node_name}Subscriber"
        for subclass in MultiverseSubscriber.__subclasses__():
            if subclass.__name__ == subscriber_name:
                return subclass(
                    topic_name=topic_name,
                    port=self.port,
                    multiverse_meta_data=self.multiverse_meta_data,
                    **self.ros_node_prop,
                )

        raise TypeError(f"Class {subscriber_name} not found.")

    def create_service(self) -> MultiverseService:
        service_name = f"{self.ros_node_name}Service"
        for subclass in MultiverseService.__subclasses__():
            if subclass.__name__ == service_name:
                return subclass(
                    port=self.port,
                    multiverse_meta_data=self.multiverse_meta_data,
                    **self.ros_node_prop,
                )

        raise TypeError(f"Class {service_name} not found.")
