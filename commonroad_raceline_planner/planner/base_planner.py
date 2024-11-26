from abc import ABC, abstractmethod
from logging import Logger

# own code base
from commonroad_raceline_planner.raceline import RaceLine

# typing
from typing import Any

from commonroad_raceline_planner.racetrack import RaceTrack


class BaseRacelinePlanner(ABC):
    """
    Base class for a raceline planner
    """

    def __init__(self, race_track: RaceTrack, config: Any) -> None:
        """
        Base class for a raceline planner
        :param lanelet_network: cr lanelet network
        :param config: planner config
        """
        self._race_track: RaceTrack = race_track
        self._config: Any = config

    @property
    @abstractmethod
    def logger(self) -> Logger:
        """
        :return: logger
        """
        pass

    @property
    def config(self) -> Any:
        """
        :return: config of the planner
        """
        return self._config

    @property
    def race_track(self):
        return self._race_track

    @abstractmethod
    def update_config(self, config: Any) -> None:
        """
        updates the planner config
        :param config: planner config
        """
        pass

    @abstractmethod
    def plan(self) -> RaceLine:
        """
        plans the raceline
        :return: cr raceline
        """
        pass
