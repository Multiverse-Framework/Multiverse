#!/usr/bin/env python3

"""Multiverse Simulator base class"""

import atexit
import logging
import time
from dataclasses import dataclass
from enum import Enum
from threading import Thread
from typing import Optional, Dict, List

import numpy


class MultiverseSimulatorState(Enum):
    """Multiverse Simulator State Enum"""
    STOPPED = 0
    PAUSED = 1
    RUNNING = 2


class MultiverseSimulatorStopReason(Enum):
    """Multiverse Simulator Stop Reason"""
    STOP = 0
    MAX_REAL_TIME = 1
    MAX_SIMULATION_TIME = 2
    MAX_NUMBER_OF_STEPS = 3
    VIEWER_IS_CLOSED = 4
    OTHER = 5


@dataclass
class MultiverseSimulatorConstraints:
    max_real_time: float = None
    max_simulation_time: float = None
    max_number_of_steps: int = None


class MultiverseRenderer:
    """Base class for Multiverse Renderer"""

    _is_running: bool = False

    def __init__(self):
        self._is_running = True
        atexit.register(self.close)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def is_running(self) -> bool:
        """Check if the renderer is running"""
        return self._is_running

    def sync(self):
        """Update the renderer"""
        pass

    def close(self):
        """Close the renderer"""
        self._is_running = False


@dataclass
class MultiverseAttribute:
    """Base class for Multiverse Attribute"""

    name: str
    """Name of the attribute"""
    value: numpy.ndarray
    """Value of the attribute"""


class MultiverseViewer:
    """Base class for Multiverse Viewer"""

    def __init__(
            self,
            send_objects: Optional[Dict[str, List[MultiverseAttribute]]] = None,
            receive_objects: Optional[Dict[str, List[MultiverseAttribute]]] = None,
    ):
        self._send_objects = send_objects or {}
        self._send_data = self._initialize_data(self._send_objects)

        self._receive_objects = receive_objects or {}
        self._receive_data = self._initialize_data(self._receive_objects)

    @staticmethod
    def _initialize_data(objects: Dict[str, List[MultiverseAttribute]]) -> numpy.ndarray:
        """
        Flatten attribute values into a NumPy array.

        :param objects: Dict[str, List[MultiverseAttribute]], objects with attributes
        :return: numpy.ndarray, flattened attribute values
        """
        return numpy.array([i for attrs in objects.values() for attr in attrs for i in attr.value])

    @property
    def send_objects(self) -> Dict[str, List[MultiverseAttribute]]:
        self._update_objects(self._send_objects, self.send_data)
        return self._send_objects

    @property
    def receive_objects(self) -> Dict[str, List[MultiverseAttribute]]:
        self._update_objects(self._receive_objects, self.receive_data)
        return self._receive_objects

    @staticmethod
    def _update_objects(objects: Dict[str, List[MultiverseAttribute]], data: numpy.ndarray):
        """
        Update object attribute values from the data array.

        :param objects: Dict[str, List[MultiverseAttribute]], objects with attributes
        """
        i = 0
        for attributes in objects.values():
            for attr in attributes:
                attr.value = data[i:i + len(attr.value)]
                i += len(attr.value)

    @property
    def send_data(self) -> numpy.ndarray:
        return self._send_data

    @send_data.setter
    def send_data(self, data: numpy.ndarray):
        if len(data) != len(self._send_data):
            raise ValueError("Data length mismatch with send_objects.")
        self._send_data[:] = data

    @property
    def receive_data(self) -> numpy.ndarray:
        return self._receive_data

    @receive_data.setter
    def receive_data(self, data: numpy.ndarray):
        if len(data) != len(self._receive_data):
            raise ValueError("Data length mismatch with receive_objects.")
        self._receive_data[:] = data


class MultiverseSimulator:
    """Base class for Multiverse Simulator"""

    name: str = "Multiverse Simulation"
    """Name of the simulator"""

    ext: str = ""
    """Extension of the simulator description file"""

    simulation_thread: Thread = None
    """Simulation thread, run step() method in this thread"""

    logger: logging.Logger = logging.getLogger(__name__)
    """Logger for the simulator"""

    def __init__(self, viewer: Optional[MultiverseViewer] = None, **kwargs):
        """
        Initialize the simulator with the viewer and the following keyword arguments:

        :param viewer: MultiverseViewer, viewer for the simulator
        :param kwargs: step_size, headless, real_time_factor
        """
        self._headless = kwargs.get("headless", False)
        self._real_time_factor = kwargs.get("real_time_factor", 1.0)
        self._step_size = kwargs.get("step_size", 1E-3)
        self._current_number_of_steps = 0
        self._start_real_time = self.current_real_time
        self._state = MultiverseSimulatorState.STOPPED
        self._stop_reason = None
        self._viewer = viewer
        self._renderer = MultiverseRenderer()
        self._current_view_time = self.current_real_time
        atexit.register(self.stop)

    def start(self,
              run_in_thread: bool = True,
              constraints: MultiverseSimulatorConstraints = None,
              time_out_in_seconds: float = 10.0):
        """
        Start the simulator, if run_in_thread is True, run the simulator in a thread until the constraints are met

        :param constraints: MultiverseSimulatorConstraints, constraints for stopping the simulator
        :param run_in_thread: bool, True to run the simulator in a thread
        :param constraints: MultiverseSimulatorConstraints, constraints for stopping the simulator
        :param time_out_in_seconds: float, timeout for starting the renderer
        """
        self.start_callback()
        for i in range(int(10 * time_out_in_seconds)):
            if self.renderer.is_running():
                break
            time.sleep(0.1)
            if i % 10 == 0:
                self.log_info(f"Waiting for {self.renderer.__name__} to start")
        else:
            self.log_error(f"{self.renderer.__name__} is not running")
            return
        self._current_number_of_steps = 0
        self._start_real_time = self.current_real_time
        self._state = MultiverseSimulatorState.RUNNING
        self._stop_reason = None
        if run_in_thread:
            self.simulation_thread = Thread(target=self.run, args=(constraints,))
            self.simulation_thread.start()

    def run(self,
            constraints: MultiverseSimulatorConstraints = None):
        """
        Run the simulator while the state is RUNNING or until the constraints are met

        :param constraints: MultiverseSimulatorConstraints, constraints for stopping the simulator
        """
        with self.renderer:
            while self.state != MultiverseSimulatorState.STOPPED:
                self._stop_reason = self.should_stop(constraints)
                if self.stop_reason is not None:
                    self._state = MultiverseSimulatorState.STOPPED
                    break
                if self.state == MultiverseSimulatorState.RUNNING:
                    if self.current_simulation_time == 0.0:
                        self.reset()
                    if self.real_time_factor > 0:
                        real_time_pass = self.current_real_time - self.start_real_time
                        simulation_time_pass = self.current_simulation_time * self.real_time_factor
                        delta_time = simulation_time_pass - real_time_pass
                        if delta_time <= self.step_size:
                            self.step()
                        if delta_time > self.step_size * 10:
                            self.log_warning(
                                f"Real time is {delta_time} seconds ({delta_time / self.step_size} step_size) behind simulation time")
                        elif delta_time < -self.step_size * 10:
                            self.log_warning(
                                f"Real time is {-delta_time} seconds ({-delta_time / self.step_size} step_size) ahead of simulation time")
                    else:
                        self.step()
                elif self.state == MultiverseSimulatorState.PAUSED:
                    self.pause_callback()
                self.run_callback()
        self.stop_callback()

    def step(self):
        """Step the simulator"""
        if self._viewer is not None:
            self.write_data(in_data=self._viewer.send_data)
        self.step_callback()
        if self._viewer is not None:
            self.read_data(out_data=self._viewer.receive_data)
        self._current_number_of_steps += 1

    def write_data(self, in_data: numpy.ndarray):
        """
        Write data to the simulator

        :param in_data: numpy.ndarray, data to write
        """
        raise NotImplementedError("write_data method is not implemented")

    def read_data(self, out_data: numpy.ndarray):
        """
        Read data from the simulator

        :param out_data: numpy.ndarray, data to read
        """
        raise NotImplementedError("read_data method is not implemented")

    def stop(self):
        """Stop the simulator"""
        if self.renderer.is_running():
            self.renderer.close()
        self._state = MultiverseSimulatorState.STOPPED
        if self.simulation_thread is not None and self.simulation_thread.is_alive():
            self.simulation_thread.join()
        self._stop_reason = MultiverseSimulatorStopReason.STOP

    def pause(self):
        """Pause the simulator"""
        if self.state != MultiverseSimulatorState.RUNNING:
            self.log_warning("Cannot pause when the simulator is not running")
        else:
            self._state = MultiverseSimulatorState.PAUSED

    def unpause(self):
        """Unpause the simulator and run the simulator"""
        if self.state == MultiverseSimulatorState.PAUSED:
            self.unpause_callback()
            self._state = MultiverseSimulatorState.RUNNING
        else:
            self.log_warning("Cannot unpause when the simulator is not paused")

    def reset(self):
        """
        Reset the simulator, set the start_real_time to current_real_time, current_simulate_time to 0.0,
        current_number_of_steps to 0, and run the simulator
        """
        self.reset_callback()
        self._current_number_of_steps = 0
        self._start_real_time = self.current_real_time

    def should_stop(self,
                    constraints: MultiverseSimulatorConstraints = None) -> Optional[MultiverseSimulatorStopReason]:
        """
        :param constraints: MultiverseSimulatorConstraints, constraints for stopping the simulator
        :return: bool, True if the simulator should stop, False otherwise
        """
        if constraints is not None:
            if constraints.max_real_time is not None and self.current_real_time - self.start_real_time >= constraints.max_real_time:
                self.log_info(f"Stopping simulation because max_real_time [{constraints.max_real_time}] reached")
                return MultiverseSimulatorStopReason.MAX_REAL_TIME
            if constraints.max_simulation_time is not None and self.current_simulation_time >= constraints.max_simulation_time:
                self.log_info(
                    f"Stopping simulation because max_simulation_time [{constraints.max_simulation_time}] reached")
                return MultiverseSimulatorStopReason.MAX_SIMULATION_TIME
            if constraints.max_number_of_steps is not None and self.current_number_of_steps >= constraints.max_number_of_steps:
                self.log_info(
                    f"Stopping simulation because max_number_of_steps [{constraints.max_number_of_steps}] reached")
                return MultiverseSimulatorStopReason.MAX_NUMBER_OF_STEPS
        return self.should_stop_callback()

    def start_callback(self):
        self._current_simulation_time = 0.0
        self._renderer = MultiverseRenderer()

    def run_callback(self):
        if self.current_real_time - self._current_view_time > 1.0 / 60.0:
            self._current_view_time = self.current_real_time
            self.renderer.sync()

    def step_callback(self):
        self._current_simulation_time += self.step_size

    def stop_callback(self):
        if self.renderer.is_running():
            self.renderer.close()

    def pause_callback(self):
        self._start_real_time += self.current_real_time - self.current_simulation_time - self.start_real_time

    def unpause_callback(self):
        pass

    def reset_callback(self):
        self._current_simulation_time = 0.0

    def should_stop_callback(self) -> Optional[MultiverseSimulatorStopReason]:
        return None if self.renderer.is_running() else MultiverseSimulatorStopReason.VIEWER_IS_CLOSED

    def log_info(self, message: str):
        self.logger.info(f"[{self.name}] {message}")

    def log_warning(self, message: str):
        self.logger.warning(f"[{self.name}] {message}")

    def log_error(self, message: str):
        self.logger.error(f"[{self.name}] {message}")

    @property
    def headless(self) -> bool:
        return self._headless

    @property
    def real_time_factor(self) -> float:
        return self._real_time_factor

    @property
    def step_size(self) -> float:
        return self._step_size

    @property
    def state(self) -> MultiverseSimulatorState:
        return self._state

    @property
    def stop_reason(self) -> MultiverseSimulatorStopReason:
        return self._stop_reason

    @property
    def start_real_time(self) -> float:
        return self._start_real_time

    @property
    def current_real_time(self) -> float:
        return time.time()

    @property
    def current_simulation_time(self) -> float:
        return self._current_simulation_time

    @property
    def current_number_of_steps(self) -> int:
        return self._current_number_of_steps

    @property
    def renderer(self) -> MultiverseRenderer:
        return self._renderer
