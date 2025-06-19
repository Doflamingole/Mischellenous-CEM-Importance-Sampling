import sys
import os
import glob
import queue

def init_carla_env():
    # ==============================================================================
    # -- Find CARLA module ---------------------------------------------------------
    # ==============================================================================
    try:
        sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except IndexError:
        pass

    # ==============================================================================
    # -- Add PythonAPI for release mode --------------------------------------------
    # ==============================================================================
    try:
        sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
    except IndexError:
        pass

# import carla
# import random
# from typing import Tuple, Mapping
# import render_utils, carlaUtils
# import numpy as np
# from carla import Location, Rotation, Vehicle, ColorConverter, Vector3D, Client, World
