import abc
from typing import Union, Tuple
from collections import OrderedDict

import numpy as np

class BaseRGBDCamera(abc.ABC):

    def __init__(self):
        self._color_frame = None  # np.array
        self._depth_frame = None  # np.array
        self._intrinsics = None  # dict

    @abc.abstractmethod
    def poll(self) -> None:
        """Polls camera to compute new frame
        """
        pass

    @abc.abstractmethod
    def _compute_intrinsics(self) -> Tuple[np.ndarray, np.ndarray, float]:
        """Computes camera intrinsics

        Returns
        -------
        np.ndarray
            3x3 matrix containging focal distances and center distances as:
            `[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`

        np.ndarray
            1-D array containing distorssion coefficients

        float
            Depth scale
        """
        pass

    @abc.abstractmethod
    def shutdown(self) -> None:
        """Shutdowns camera
        """
        pass

    @property
    def intrinsics(self) -> OrderedDict:
        """Camera intrinsics

        Returns
        -------
        OrderedDict
            Dictionary containing camera intrinsics with keys: `intrinsics, coefficients, depth_scale`
        """
        if self._intrinsics is None:
            intrinsics_matrix, distorssion_coefficients, depth_scale = self._compute_intrinsics()

            self._intrinsics = OrderedDict({
                "intrinsics": intrinsics_matrix.astype(np.float32),
                "coefficients": distorssion_coefficients.astype(np.float32),
                "depth_scale": depth_scale
            })

        return self._intrinsics

    @property
    def depth_frame(self) -> Union[np.ndarray, None]:
        """Last computed depth frame

        Returns
        -------
        Union[np.ndarray, None]
            Last computed depth frame (np.uint16) or `None` if none was computed yet.
        """
        result = None
        if self._depth_frame is not None:
            result = self._depth_frame.copy()

        return result

    @property
    def color_frame(self) -> Union[np.ndarray, None]:
        """Last computed color frame

        Returns
        -------
        Union[np.ndarray, None]
            Last computed color frame in BGR format (np.uint8) or `None` if none was computed yet.
        """
        result = None
        if self._color_frame is not None:
            result = self._color_frame.copy()

        return result
