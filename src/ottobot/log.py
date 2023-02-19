import logging

import rospy


STDOUT_FORMAT = "%(asctime)s %(name)-4s %(levelname)-4s %(funcName)-8s %(message)s"


def set_root_logger(verbose: bool = False):
    log_level = logging.DEBUG if verbose else logging.INFO
    logger = logging.getLogger()
    logger.setLevel(log_level)
    handler = logging.StreamHandler()
    handler.setLevel(log_level)
    logger.addHandler(handler)
    formatter = logging.Formatter(STDOUT_FORMAT)
    handler.setFormatter(formatter)


class ConnectPythonLoggingToROS(logging.Handler):

    level_map = {
        logging.DEBUG: rospy.logdebug,
        logging.INFO: rospy.loginfo,
        logging.WARNING: rospy.logwarn,
        logging.ERROR: rospy.logerr,
        logging.CRITICAL: rospy.logfatal
    }

    def emit(self, record):
        try:
            self.level_map[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))


def route_logger_to_ros(logger_name):
    '''Re-routes a Python logging.logger to the ROS logging infrastructure.
    Without using this, once `rospy.init_node()` has been called, any use of `logging` occurs silently.
    Example:
        rospy.init_node('my_node')
        route_logger_to_ros('my_custom_library')
        # In an imported library:
        logger = logging.getLogger('my_custom_library)
        logger.info('This message gets routed to ROS logging if a ROS node was initialized in this process.')
    '''
    logging.getLogger(logger_name).addHandler(ConnectPythonLoggingToROS())
