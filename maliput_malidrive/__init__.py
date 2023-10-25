import os

def get_plugin_path():
    # return this location. It is expected to be the plugin located in this same location
    return os.path.dirname(os.path.abspath(__file__))
