from abc import ABC, abstractmethod
from pathlib import Path
from configparser import ConfigParser
import os

# typing
from typing import Union, Any


class BaseConfigFactory(ABC):
    """
    Base factory class
    """
    def __init__(self):
        self._parser = ConfigParser()


    @staticmethod
    def _sanity_check_ini(
            path_to_racecar_ini: Union[Path, str]
    ) -> bool:
        """
        Check sanity of path
        :param path_to_racecar_ini:
        :return:
        """
        sanity: bool = True
        if(
            not
                os.path.exists(path_to_racecar_ini)
                and os.path.isabs(path_to_racecar_ini)
                and path_to_racecar_ini.split('.')[-1] == ".ini"
        ):
            sanity = False

        return sanity







