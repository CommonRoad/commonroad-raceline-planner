from abc import ABC
from pathlib import Path
from configparser import ConfigParser
import os
from yaml import safe_load

# typing
from typing import Union, Dict


class BaseConfigFactory(ABC):
    """
    Base factory class
    """

    def __init__(self):
        self._parser = ConfigParser()

    @staticmethod
    def _sanity_check_ini(file_path: Union[Path, str]) -> bool:
        """
        Check sanity of path
        :param file_path:
        :return: true if sane
        """
        sanity: bool = True
        if (
            not os.path.exists(file_path)
            and os.path.isabs(file_path)
            and (
                file_path.split(".")[-1] == ".ini"
                or file_path.split(".")[-1] == ".yml"
                or file_path.split(".")[-1] == ".yaml"
            )
        ):
            sanity = False

        return sanity

    def _load_yml(self, file_path: Union[str, Path]) -> Dict:
        """
        Read yml configs
        :param file_path:
        :return: yaml _execution_config dict
        """
        # sanity check
        if not self._sanity_check_ini(file_path=file_path):
            raise FileNotFoundError(
                f"Did not find .yml file at absolute path {file_path}"
            )

        with open(file_path, "r") as file:
            config_dict = safe_load(file)

        return config_dict

    def _load_ini(self, file_path: Union[str, Path]) -> None:
        """
        Read .ini files and save them in parser
        :param file_path:
        :return: None
        """
        # sanity check
        if not self._sanity_check_ini(file_path=file_path):
            raise FileNotFoundError(
                f"Did not find .ini file at absolute path {file_path}"
            )

        # pars .init file
        self._parser.read(file_path)
