#!/usr/bin/env python3

import os

import numpy
from PIL import Image


class TextureBuilder:
    file_path: str

    def __init__(self, file_path: str) -> None:
        self._file_path = file_path

    @property
    def file_path(self):
        return self._file_path

    @property
    def rgb(self):
        if not os.path.exists(self._file_path):
            raise ValueError(f"File {self._file_path} does not exist.")

        image = Image.open(self._file_path)
        img_rgb = image.convert('RGB')
        return numpy.array(img_rgb)

    @rgb.setter
    def rgb(self, rgb: numpy.ndarray):
        img = Image.fromarray(rgb, 'RGB')
        if not os.path.exists(os.path.dirname(self.file_path)):
            os.makedirs(os.path.dirname(self.file_path))
        img.save(self.file_path)
