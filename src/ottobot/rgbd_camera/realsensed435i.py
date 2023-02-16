import logging
import time
from typing import Tuple, Union
from pathlib import Path
import json

import pyrealsense2 as rs
import numpy as np

from ottobot.rgbd_camera.base_rgbdcamera import BaseRGBDCamera


logger = logging.getLogger(__name__)


class RealSenseD453iError(Exception):
    pass


class RealSenseD435i(BaseRGBDCamera):

    _ALIGN_TO_VALID_OPTIONS = ["color", "depth"]

    # TODO: Fix doc to get correct pyrealsense types on type hints
    def __init__(
        self, context: rs.context, fps: int, height: int, width: int, device: rs.device, align_to: str = "depth",
        rs_viewer_config: Union[str, Path] = None
    ):

        assertion_msg = "Got invalid 'align_to' option '{}', valid values are '{}'".format(
            align_to, self._ALIGN_TO_VALID_OPTIONS
        )
        assert align_to.lower() in self._ALIGN_TO_VALID_OPTIONS, assertion_msg

        super(RealSenseD435i, self).__init__()

        self._fps = fps
        self._height = height
        self._width = width
        self._align_to = align_to

        self._rs_context = context
        self._device = device
        self._device_id = self._device.get_info(rs.camera_info.serial_number)

        logger.info("Resetting '{}'..".format(self._device_id))
        self._device.hardware_reset()
        logger.info("DONE")

        self._rs_align = None

        if rs_viewer_config is not None:
            self._rs_config_file = Path(rs_viewer_config).resolve()

            if not self._rs_config_file.exists():
                logger.error(
                    "Could not find RealSense Viewer config file at '{}'. Skipping configuration from file".format(
                        str(self._rs_config_file)
                    )
                )

                self._rs_config_file = None

        self._rs_images_pipeline = rs.pipeline(self._rs_context)
        config = rs.config()

        self._enable_images_streams(config)

    def _enable_images_streams(self, config: rs.config) -> None:

        logger.info("Configuring Depth and Color streams..")

        if self._device_id is not None:
            config.enable_device(self._device_id)

        config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
        config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)

        if self._align_to == "depth":
            self._align = rs.align(rs.stream.depth)

        else:
            self._align = rs.align(rs.stream.color)

        # Start image streams
        streams_up = False
        attemps = 5

        for _ in range(attemps):
            try:
                self._rs_images_profile = self._rs_images_pipeline.start(config)
                streams_up = True

            except Exception as e:
                logger.warning("Got exception while trying to setup images streams '{}', trying again..".format(
                    e
                ))

            if streams_up:
                break

        if not streams_up:
            raise RuntimeError("Could not start images streams after '{}' attemps".format(attemps))

        logger.info("Started pipeline, waiting for first frames..")

        # Eat some frames to allow auto-exposure to settle
        for _ in range(5):
            try:
                self._rs_images_pipeline.wait_for_frames(10000)

            except Exception as e:
                logger.warning(e)

        logger.info("Images streams are up, waiting 2 seconds to allow camera to warm up")
        time.sleep(2)

        # Load config file
        if self._rs_config_file is not None:

            logger.info("Loading Advanced configurations from file..")

            with self._rs_config_file.open("r") as fp:
                rs_viewer_data = json.load(fp)

            try:

                device_data = rs_viewer_data["parameters"]

                advnc_mode = rs.rs400_advanced_mode(self._device)
                advnc_mode.load_json(json.dumps(device_data))

            except Exception as e:
                raise RealSenseD453iError("Unexpected error when loading camera config from '{}': '{}'".format(
                    str(self._rs_config_file), e
                ))

            logger.info("Configuration file loaded")

        logger.info("DONE")

    def _compute_intrinsics(self) -> Tuple[np.ndarray, np.ndarray, float]:

        frame = None

        depth_scale = self._rs_images_profile.get_device().first_depth_sensor().get_depth_scale()

        if self._align_to == "depth":
            frame = self._rs_images_profile.get_stream(rs.stream.depth)

        else:
            frame = self._rs_images_profile.get_stream(rs.stream.color)

        intrinsics = frame.as_video_stream_profile().intrinsics

        # Create camera matrix
        # [fx 0  cx]
        # [0  fy cy]
        # [0  0  1]
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0,             0,              1]
        ], dtype=np.float32)

        coeffs = np.array(intrinsics.coeffs, dtype=np.float32)

        return camera_matrix, coeffs, depth_scale

    def shutdown(self) -> None:
        if self._rs_images_pipeline is not None:
            self._rs_images_pipeline.stop()

    def poll(self) -> None:
        self._color_frame = None
        self._depth_frame = None

        try:
            frames = self._rs_images_pipeline.wait_for_frames(int((1 / self._fps) * 1000))

        except Exception as e:
            logger.error(e)
            return

        aligned_frames = self._align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Convert depth to 16bit array, BGR into 8bit array
        self._depth_frame = np.asanyarray(depth_frame.get_data(), dtype=np.uint16)
        self._color_frame = np.asanyarray(color_frame.get_data(), dtype=np.uint8)
