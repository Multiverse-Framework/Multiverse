#!/usr/bin/env python3

"""Multiverse Simulator base class"""

import atexit
import logging
import time
from dataclasses import dataclass
from enum import Enum
from functools import partial
from threading import Thread
from typing import Optional, Dict, List, Tuple, Any, Callable

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

    default_value: numpy.ndarray
    """Default value of the attribute"""
    _values: numpy.ndarray = None
    """Values of the attribute"""

    def __init__(self, default_value: numpy.ndarray):
        self.default_value = default_value

    def initialize_data(self, number_of_envs: int):
        """
        Initialize the data for the attribute

        :param number_of_envs: int, number of environments
        """
        self._values = numpy.array([self.default_value for _ in range(number_of_envs)])

    @property
    def values(self):
        if self._values is None:
            raise ValueError("Values are not set, call initialize_data() first.")
        return self._values


class MultiverseLogger:
    """Base class for Multiverse Logger"""

    def __init__(self, objects: Dict[str, Dict[str, MultiverseAttribute]]):
        self._data_size = 1
        for object_data in objects.values():
            for attribute_values in object_data.values():
                self._data_size += len(attribute_values.default_value)
        self._objects = objects
        self._data = numpy.array([])
        self._start_time = time.time()

    def log_data(self, new_data: numpy.ndarray):
        if new_data.size != self._data_size - 1:
            raise ValueError("New data size does not match existing data size.")
        new_data = numpy.append([time.time()], new_data)
        self._data = numpy.append(self._data, new_data)

    def save_data(self, save_file_path: str):
        data = {}
        number_of_data = int(len(self._data) / self._data_size)
        data['step'] = numpy.arange(0, number_of_data)
        data['time'] = self._data[0::self._data_size]
        data_adr = 1
        for object_name, object_data in self.objects.items():
            for attribute_name, attribute_values in object_data.items():
                if len(attribute_values.default_value) == 1:
                    data_name = f"{object_name}_{attribute_name}"
                    data[data_name] = self.data[data_adr::self._data_size]
                    data_adr += 1
                else:
                    for i in range(len(attribute_values.default_value)):
                        data_name = f"{object_name}_{attribute_name}_{i}"
                        data[data_name] = self.data[data_adr::self._data_size]
                        data_adr += 1

        import pandas as pd

        # Create a DataFrame
        df = pd.DataFrame(data)

        # Writing to CSV, index=False to avoid writing row numbers
        print(f"Saving data to {save_file_path}")
        df.to_csv(save_file_path, index=False)

    @property
    def objects(self):
        return self._objects

    @property
    def data(self):
        return self._data

    @property
    def start_time(self):
        return self._start_time


class MultiverseViewer:
    """Base class for Multiverse Viewer"""

    logger: Optional[MultiverseLogger] = None

    def __init__(
            self,
            write_objects: Optional[Dict[str, Dict[str, numpy.ndarray | List[float]]]] = None,
            read_objects: Optional[Dict[str, Dict[str, numpy.ndarray | List[float]]]] = None,
            logging_interval: float = -1
    ):
        self._write_objects = self.from_array(write_objects) if write_objects is not None else {}
        self._write_data = numpy.array([])

        self._read_objects = self.from_array(read_objects) if read_objects is not None else {}
        self._read_data = numpy.array([])

        self._logging_interval = logging_interval
        if self.logging_interval > 0:
            self.logger = MultiverseLogger(self.read_objects)

    @staticmethod
    def from_array(data: Dict[str, Dict[str, numpy.ndarray | List[float]]]) \
            -> Dict[str, Dict[str, MultiverseAttribute]]:
        """
        Convert the data array to MultiverseAttribute objects

        :param data: Dict[str, Dict[str, numpy.ndarray | List[float]]], data array
        :return: Dict[str, Dict[str, MultiverseAttribute]], MultiverseAttribute objects
        """
        return {key: {key2: MultiverseAttribute(default_value=value)
                      for key2, value in value.items()} for key, value in data.items()}

    def initialize_data(self, number_of_envs: int) -> "MultiverseViewer":
        """
        Initialize the data for the viewer

        :param number_of_envs: int, number of environments
        """
        self._write_data = numpy.array([self._initialize_data(self._write_objects)
                                        for _ in range(number_of_envs)])
        self._read_data = numpy.array([self._initialize_data(self._read_objects)
                                       for _ in range(number_of_envs)])
        for objects in [self._write_objects, self._read_objects]:
            for attrs in objects.values():
                for attr in attrs.values():
                    attr.initialize_data(number_of_envs)
        return self

    @staticmethod
    def _initialize_data(objects: Dict[str, Dict[str, MultiverseAttribute]]) -> numpy.ndarray:
        """
        Flatten attribute values into a NumPy array.

        :param objects: Dict[str, Dict[str, MultiverseAttribute]], objects with attributes
        :return: numpy.ndarray, flattened attribute values
        """
        return numpy.array([i for attrs in objects.values() for attr in attrs.values() for i in attr.default_value])

    @property
    def write_objects(self) -> Dict[str, Dict[str, MultiverseAttribute]]:
        self._update_objects_from_data(self._write_objects, self.write_data)
        return self._write_objects

    @write_objects.setter
    def write_objects(self, send_objects: Dict[str, Dict[str, numpy.ndarray | List[float] | MultiverseAttribute]]):
        number_of_envs = self.write_data.shape[0]
        self._write_objects, self._write_data = (
            self._get_objects_and_data_from_target_objects(send_objects, number_of_envs))
        assert self.write_data.shape[0] == self.read_data.shape[0]

    @property
    def read_objects(self) -> Dict[str, Dict[str, MultiverseAttribute]]:
        self._update_objects_from_data(self._read_objects, self.read_data)
        return self._read_objects

    @read_objects.setter
    def read_objects(self, objects: Dict[str, Dict[str, numpy.ndarray | List[float] | MultiverseAttribute]]):
        number_of_envs = self.read_data.shape[0]
        self._read_objects, self._read_data = (
            self._get_objects_and_data_from_target_objects(objects, number_of_envs))
        assert self.read_data.shape[0] == self.write_data.shape[0]
        if self.logging_interval > 0:
            self.logger = MultiverseLogger(self.read_objects)

    @staticmethod
    def _get_objects_and_data_from_target_objects(
            target_objects: Dict[str, Dict[str, numpy.ndarray | List[float] | MultiverseAttribute]],
            number_of_envs: int) \
            -> Tuple[Dict[str, Dict[str, MultiverseAttribute]], numpy.ndarray]:
        """
        Update object attribute values from the target objects.

        :param target_objects: Dict[str, Dict[str, numpy.ndarray | List[float] | MultiverseAttribute]], target objects
        :param number_of_envs: int, number of environments
        """
        if any(isinstance(value, (numpy.ndarray, list)) for values in target_objects.values() for value in
               values.values()):
            objects = MultiverseViewer.from_array(target_objects) if target_objects is not None else {}
        else:
            objects = target_objects
        for attrs in objects.values():
            for attr in attrs.values():
                if attr._values is None:
                    attr.initialize_data(number_of_envs)
        data = numpy.array([[value for attrs in objects.values()
                             for attr in attrs.values()
                             for value in attr.values[env_id]] for env_id in range(number_of_envs)])
        return objects, data

    @staticmethod
    def _update_objects_from_data(objects: Dict[str, Dict[str, MultiverseAttribute]], data: numpy.ndarray):
        """
        Update object attribute values from the data array.

        :param objects: Dict[str, List[MultiverseAttribute]], objects with attributes
        """
        for i, values in enumerate(data):
            j = 0
            for obj_name, attrs in objects.items():
                for name, attr in attrs.items():
                    attr.values[i] = values[j:j + len(attr.default_value)]
                    j += len(attr.default_value)

    @property
    def write_data(self) -> numpy.ndarray:
        return self._write_data

    @write_data.setter
    def write_data(self, data: numpy.ndarray):
        if data.shape != self._write_data.shape:
            raise ValueError(
                f"Data length mismatch with write_objects, expected {self._write_data.shape}, got {data.shape}")
        self._write_data[:] = data

    @property
    def read_data(self) -> numpy.ndarray:
        return self._read_data

    @read_data.setter
    def read_data(self, data: numpy.ndarray):
        if data.shape != self._read_data.shape:
            raise ValueError(
                f"Data length mismatch with read_objects, expected {self._read_data.shape}, got {data.shape}")
        self._read_data[:] = data

    @property
    def logging_interval(self) -> float:
        return self._logging_interval


@dataclass
class MultiverseCallbackResult:
    """Multiverse Function Result Enum"""

    class OutType(str, Enum):
        MUJOCO = "mujoco"
        PYBULLET = "pybullet"
        ISAACSIM = "isaacsim"

    class ResultType(Enum):
        SUCCESS_WITHOUT_EXECUTION = 0
        SUCCESS_AFTER_EXECUTION_ON_MODEL = 1
        SUCCESS_AFTER_EXECUTION_ON_DATA = 2
        FAILURE_WITHOUT_EXECUTION = 3
        FAILURE_BEFORE_EXECUTION_ON_MODEL = 4
        FAILURE_AFTER_EXECUTION_ON_MODEL = 5
        FAILURE_BEFORE_EXECUTION_ON_DATA = 6
        FAILURE_AFTER_EXECUTION_ON_DATA = 7

    type: ResultType
    """Result type"""
    info: str = None
    """Information about the result"""
    result: Any = None
    """Result of the callback"""

    def __call__(self):
        self.result = self.result()
        return self


class MultiverseCallback:
    """Base class for Multiverse Callback"""

    def __init__(self, callback: Callable):
        """
        Initialize the function with the callback

        :param callback: Callable, callback function, must return MultiverseCallbackResult
        """
        self._call = callback
        self.__name__ = callback.__name__

    def __call__(self, *args, render: bool = True, **kwargs):
        result = self._call(*args, **kwargs)
        if not isinstance(result, MultiverseCallbackResult):
            raise TypeError("Callback function must return MultiverseCallbackResult")
        simulator = args[0]
        if not isinstance(simulator, MultiverseSimulator):
            raise TypeError("First argument must be of type MultiverseSimulator")
        if render:
            simulator.renderer.sync()
        return result


class MultiverseSimulator:
    """Base class for Multiverse Simulator"""

    name: str = "Multiverse Simulation"
    """Name of the simulator"""

    ext: str = ""
    """Extension of the simulator description file"""

    simulation_thread: Thread = None
    """Simulation thread, run step() method in this thread"""

    render_thread: Thread = None
    """Render thread, run render() method in this thread"""

    logger: logging.Logger = logging.getLogger(__name__)
    """Logger for the simulator"""

    class_level_callbacks: List[MultiverseCallback] = []
    """Class level callback functions"""

    instance_level_callbacks: List[MultiverseCallback] = None
    """Instance level callback functions"""

    def __init__(self,
                 viewer: Optional[MultiverseViewer] = None,
                 number_of_envs: int = 1,
                 headless: bool = False,
                 real_time_factor: float = 1.0,
                 step_size: float = 1E-3,
                 callbacks: List[MultiverseCallback] = None,
                 **kwargs):
        """
        Initialize the simulator with the viewer and the following keyword arguments:

        :param viewer: MultiverseViewer, viewer for the simulator
        :param number_of_envs: int, number of environments
        :param headless: bool, True to run the simulator in headless mode
        :param real_time_factor: float, real time factor
        :param step_size: float, step size
        :param callbacks: List[MultiverseCallback], list of callback functions
        """
        self._headless = headless
        self._real_time_factor = real_time_factor
        self._step_size = step_size
        self._current_number_of_steps = 0
        self._start_real_time = self.current_real_time
        self._state = MultiverseSimulatorState.STOPPED
        self._stop_reason = None
        self._viewer = viewer.initialize_data(number_of_envs) if viewer is not None else None
        self._renderer = MultiverseRenderer()
        self._current_render_time = self.current_real_time
        self.instance_level_callbacks = []
        if callbacks is not None:
            for func in callbacks:
                self.add_instance_callback(func)
        self._write_objects = {}
        self._read_objects = {}
        self._write_ids = {}
        self._read_ids = {}
        atexit.register(self.stop)

    @property
    def callbacks(self):
        return {callback.__name__: partial(callback, self) for callback in
                [*self.class_level_callbacks, *self.instance_level_callbacks]}

    def start(self,
              simulate_in_thread: bool = True,
              render_in_thread: bool = False,
              constraints: MultiverseSimulatorConstraints = None,
              time_out_in_seconds: float = 10.0):
        """
        Start the simulator, if run_in_thread is True, run the simulator in a thread until the constraints are met

        :param constraints: MultiverseSimulatorConstraints, constraints for stopping the simulator
        :param simulate_in_thread: bool, True to simulate the simulator in a thread
        :param render_in_thread: bool, True to render the simulator in a thread
        :param constraints: MultiverseSimulatorConstraints, constraints for stopping the simulator
        :param time_out_in_seconds: float, timeout for starting the renderer
        """
        self.start_callback()
        self.reset()
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
        if simulate_in_thread:
            self.simulation_thread = Thread(target=self.run, args=(constraints,))
            self.simulation_thread.start()
        if not self.headless and render_in_thread:
            def render():
                with self.renderer:
                    while self.renderer.is_running():
                        self.renderer.sync()
                        time.sleep(1.0 / 60.0)

            self.render_thread = Thread(target=render)
            self.render_thread.start()

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
                    if self.current_simulation_time == 0.0 or not numpy.isclose(
                            self.current_number_of_steps * self.step_size, self.current_simulation_time):
                        self.reset()
                    if self.real_time_factor > 0:
                        real_time_pass = self.current_real_time - self.start_real_time
                        simulation_time_pass = self.current_simulation_time * self.real_time_factor
                        delta_time = simulation_time_pass - real_time_pass
                        if delta_time <= self.step_size:
                            self.step()
                        if delta_time > self.step_size * self.real_time_factor * 10:
                            self.log_warning(
                                f"Real time is {delta_time} seconds ({delta_time / self.step_size} step_size) behind simulation time")
                        elif delta_time < -self.step_size * self.real_time_factor * 10:
                            self.log_warning(
                                f"Real time is {-delta_time} seconds ({-delta_time / self.step_size} step_size) ahead of simulation time")
                    else:
                        self.step()
                elif self.state == MultiverseSimulatorState.PAUSED:
                    self.pause_callback()
                if self.render_thread is None and self.current_real_time - self._current_render_time > 1.0 / 60.0:
                    self._current_render_time = self.current_real_time
                    self.render()
        self.stop_callback()

    def step(self):
        """Step the simulator"""
        self.pre_step_callback()
        if self._viewer is not None:
            self.write_data_to_simulator(write_data=self._viewer.write_data)
            self.step_callback()
            self.read_data_from_simulator(read_data=self._viewer.read_data)
            if self._viewer.logging_interval > 0.0:
                self._viewer.logger.log_data(new_data=self._viewer.read_data)
        else:
            self.step_callback()
        self._current_number_of_steps += 1

    def write_data_to_simulator(self, write_data: numpy.ndarray):
        """
        Write data to the simulator

        :param write_data: numpy.ndarray, data to write
        """
        raise NotImplementedError("write_data method is not implemented")

    def read_data_from_simulator(self, read_data: numpy.ndarray):
        """
        Read data from the simulator

        :param read_data: numpy.ndarray, data to read
        """
        raise NotImplementedError("read_data method is not implemented")

    def stop(self):
        """Stop the simulator"""
        if self.renderer.is_running():
            self.renderer.close()
        if self.render_thread is not None and self.render_thread.is_alive():
            self.render_thread.join()
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

    def render(self):
        self.renderer.sync()

    def _process_objects(self, objects, ids_dict):
        """
        Process objects for updating `read_ids` or `write_ids`.

        :param objects: Dictionary of objects and attributes.
        :param ids_dict: Dictionary to store processed IDs.
        """
        pass

    def pre_step_callback(self):
        if self._viewer is not None:
            self.__update_objects(self._viewer.write_objects, self._write_objects, self._write_ids)
            self.__update_objects(self._viewer.read_objects, self._read_objects, self._read_ids)

    def __update_objects(self, viewer_objects, cache_objects, object_ids):
        if self.__should_process_objects(viewer_objects, cache_objects):
            self._process_objects(viewer_objects, object_ids)
            cache_objects.update(viewer_objects)

    @staticmethod
    def __should_process_objects(viewer_objects, cache_objects):
        if viewer_objects == {} and cache_objects != {}:
            return True
        for name, attrs in viewer_objects.items():
            if name not in cache_objects or any(attr_name not in cache_objects[name] for attr_name in attrs):
                return True
        return False

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

    @classmethod
    def log_info(cls, message: str):
        cls.logger.info(f"[{cls.name}] {message}")

    @classmethod
    def log_warning(cls, message: str):
        cls.logger.warning(f"[{cls.name}] {message}")

    @classmethod
    def log_error(cls, message: str):
        cls.logger.error(f"[{cls.name}] {message}")

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

    @classmethod
    def add_callback(cls, callback: Callable | MultiverseCallback, callbacks: List[MultiverseCallback]):
        if not isinstance(callback, MultiverseCallback):
            if isinstance(callback, Callable):
                callback = MultiverseCallback(callback=callback)
            else:
                raise TypeError(f"Function {callback} must be an instance of MultiverseCallback or Callable, "
                                f"got {type(callback)}")
        if callback.__name__ in [callback.__name__ for callback in callbacks]:
            raise AttributeError(f"Function {callback.__name__} is already defined")
        callbacks.append(callback)
        cls.log_info(f"Function {callback.__name__} is registered")

    def add_instance_callback(self, callback: Callable | MultiverseCallback):
        self.add_callback(callback, self.instance_level_callbacks)

    @classmethod
    def add_class_callback(cls, callback: Callable | MultiverseCallback):
        cls.add_callback(callback, cls.class_level_callbacks)

    @classmethod
    def multiverse_callback(cls, callback):
        cls.add_class_callback(callback)
        return callback
