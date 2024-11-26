class ConfigException(Exception):
    def __init__(self, message):
        self.message = message


class TrackNotClosedException(Exception):
    def __init__(self, message):
        self.message = message


class TrackDataInvalidException(Exception):
    def __init__(self, message):
        self.message = message
