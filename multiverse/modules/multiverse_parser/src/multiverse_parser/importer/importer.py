import os

from ..factory.config import Configuration


class Importer:
    source_file_path: str
    config: Configuration

    def __init__(self, file_path: str, configuration: Configuration = Configuration()):
        self.source_file_path = file_path
        self.config = configuration

    def import_model(self) -> str:
        raise NotImplementedError

    @property
    def source_file_path(self) -> str:
        return self._source_file_path

    @source_file_path.setter
    def source_file_path(self, file_path: str) -> None:
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"File {file_path} not found.")
        self._source_file_path = file_path

    @property
    def config(self) -> Configuration:
        return self._config

    @config.setter
    def config(self, config: Configuration) -> None:
        if not isinstance(config, Configuration):
            raise TypeError(f"Expected {Configuration}, got {type(config)}")
        self._config = config
