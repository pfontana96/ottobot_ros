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

    _FILTERS = {
        "decimation": rs.decimation_filter(),
        "threshold": rs.threshold_filter(),
        "disparity_to_depth": rs.disparity_transform(False),
        "depth_to_disparity": rs.disparity_transform(True),
        "spatial": rs.spatial_filter(),
        "temporal": rs.temporal_filter(),
    }

    # TODO: Fix doc to get correct pyrealsense types on type hints
    def __init__(
        self, context: rs.context, fps: int, height: int, width: int, device: rs.device, align_to: str = "depth",
        rs_viewer_config: Union[str, Path] = None, exposure: int = None, gain: int = 51, depth_filters: dict = None
    ):
        assertion_msg = "Expected device to be 'D435I', got '{}'".format(
            device.get_info(rs.camera_info.name).split(" ")[-1]
        )
        assert device.get_info(rs.camera_info.name).split(" ")[-1] == "D435I", assertion_msg

        assertion_msg = "Got invalid 'align_to' option '{}', valid values are '{}'".format(
            align_to, self._ALIGN_TO_VALID_OPTIONS
        )
        assert align_to.lower() in self._ALIGN_TO_VALID_OPTIONS, assertion_msg

        super(RealSenseD435i, self).__init__()

        self._fps = fps
        self._height = height
        self._width = width
        self._align_to = align_to
        self._exposure = exposure
        self._gain = gain

        self._rs_context = context
        self._device = device
        self._device_id = self._device.get_info(rs.camera_info.serial_number)

        logger.info("{} '{}' (firmware {})".format(
            self._device.get_info(rs.camera_info.name), self._device_id,
            self._device.get_info(rs.camera_info.firmware_version)
        ))

        logger.info("Resetting..")
        self._device.hardware_reset()
        logger.info("DONE")

        self._rs_align = None

        self._rs_config_file = None
        if rs_viewer_config is not None:
            self._rs_config_file = Path(rs_viewer_config).resolve()

            if not self._rs_config_file.exists():
                logger.error(
                    "Could not find RealSense Viewer config file at '{}'. Skipping configuration from file".format(
                        str(self._rs_config_file)
                    )
                )

                self._rs_config_file = None

        self._depth_filters = None
        if depth_filters is not None:
            self._depth_filters = []
            for filter_name, filter_options in depth_filters.items():
                rs_filter = self._get_filter(filter_name=filter_name, options=filter_options)

                if rs_filter is not None:
                    self._depth_filters.append(rs_filter)

        self._rs_images_pipeline = rs.pipeline(self._rs_context)
        config = rs.config()

        self._enable_images_streams(config)

    @staticmethod
    def _get_filter(filter_name: str, options: dict = {}):

        if filter_name not in RealSenseD435i._FILTERS.keys():
            logger.warning("Got invalid filter name '{}', valid options are '{}'. Skipping..".format(
                filter_name, list(RealSenseD435i._FILTERS.keys())
            ))

            return

        logger.info("Configuring filter '{}'".format(filter_name))
        rs_filter = RealSenseD435i._FILTERS[filter_name]

        for opt_name, opt_value in options.items():
            RealSenseD435i._try_set_rs_option(rs_filter, opt_name, opt_value)

        logger.info("Filter '{}' configured".format(filter_name))
        return rs_filter

    @staticmethod
    def _try_set_rs_option(rs_object, option_name: str, value, attemps: int = 5, timeout: float = 1):

        try:
            option = getattr(rs.option, option_name)

        except AttributeError:
            logger.warning("Got invalid Realsense2 option '{}'. Skipping..".format(
                option_name
            ))
            return

        option_set = False
        for i in range(attemps):
            try:
                rs_object.set_option(option, value)
                option_set = True

            except Exception as e:
                logger.warning(
                    "[{}/{}] Got unexpected exception setting up option '{}' ({}), trying again in {:.3f} s..".format(
                        i + 1, attemps, option_name, e, timeout
                    )
                )

                time.sleep(timeout)

            if option_set:
                break

        if not option_set:
            raise RealSenseD453iError("Could not set option '{}' with value '{}' after '{}' attemps".format(
                option_name, value, attemps
            ))

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

        attemps = 5

        # NOTE: Loading advanced mode from JSON failing with 'could not set power mode'
        # # Load config file
        # if self._rs_config_file is not None:

        #     logger.info("Loading Advanced configurations from file..")

        #     with self._rs_config_file.open("r") as fp:
        #         rs_viewer_data = json.load(fp)

        #     device_data = rs_viewer_data["parameters"]

        #     advnc_mode = rs.rs400_advanced_mode(self._device)

        #     if not advnc_mode.is_enabled():

        #         raise RealSenseD435i("Advanced mode on device '{}' is disabled".format(
        #             self._device.get_info(rs.camera_info.name)
        #         ))

        #     device_configured_on_advnc_mode = False
        #     ser_dev = rs.serializable_device(self._device)
        #     for i in range(attemps):
        #         try:
        #             ser_dev.load_json(json.dumps(device_data))
        #             device_configured_on_advnc_mode = True

        #         except Exception as e:
        #             logger.warning(
        #                 "[{}/{}] Unexpected exception while loading config '{}', trying again in 1 s..".format(
        #                 i, attemps, e
        #             ))

        #             time.sleep(1)

        #         if device_configured_on_advnc_mode:
        #             break

        #     if not device_configured_on_advnc_mode:
        #         raise RealSenseD453iError("Unexpected error when loading camera config from '{}': '{}'".format(
        #             str(self._rs_config_file), e
        #         ))

        #     logger.info("Configuration file loaded")

        # Disable color autoexposure if an exposure time was set
        if self._exposure is not None:
            logger.info("Disabling auto-exposure on color sensor, setting exposure time to {} us".format(
                self._exposure
            ))
            color_sensor = self._device.first_color_sensor()
            self._try_set_rs_option(color_sensor, "enable_auto_exposure", 0)
            self._try_set_rs_option(color_sensor, "exposure", self._exposure)
            self._try_set_rs_option(color_sensor, "gain", self._gain)

            logger.info("DONE")

        # Enable IR emitter
        depth_sensor = self._device.first_depth_sensor()
        self._try_set_rs_option(depth_sensor, "emitter_enabled", 1)
        # self._try_set_rs_option(depth_sensor, "laser_power", 150.0)

        # Start image streams
        streams_up = False

        for i in range(attemps):
            try:
                self._rs_images_profile = self._rs_images_pipeline.start(config)
                streams_up = True

            except Exception as e:
                logger.warning(
                    "[{}/{}] Unexpected exception while trying to setup streams '{}', trying again in 1 s..".format(
                        i + 1, attemps, e
                    )
                )

                time.sleep(1)

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

        logger.info("Finished configuration")

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

        if self._depth_filters is not None:
            for rs_filter in self._depth_filters:
                frames = rs_filter.process(frames)

            frames = frames.as_frameset()

        aligned_frames = self._align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Convert depth to 16bit array, BGR into 8bit array
        self._depth_frame = np.asanyarray(depth_frame.get_data(), dtype=np.uint16)
        self._color_frame = np.asanyarray(color_frame.get_data(), dtype=np.uint8)
