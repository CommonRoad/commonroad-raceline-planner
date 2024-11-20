from abc import ABC, abstractmethod

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork

# own code base
from commonroad_raceline_planner.raceline import RaceLine

# typing
from typing import Any


class BaseRacelinePlanner(ABC):
    """
    Base class for a raceline planner
    """

    def __init__(
            self,
            lanelet_network: LaneletNetwork,
            config: Any
    ) -> None:
        """
        Base class for a raceline planner
        :param lanelet_network: cr lanelet network
        :param config: planner config
        """
        self._lanelet_network: LaneletNetwork = lanelet_network
        self._raceline_planning_pp = None
        self._config: Any = config

    @property
    def lanelet_network(self) -> LaneletNetwork:
        """
        :return: commonroad lanelet network
        """
        return self._lanelet_network

    @property
    def config(self) -> Any:
        """
        :return: config of the planner
        """
        return self._config

    @property
    def raceline_planning_problem(self):
        return self._raceline_planning_pp

    @abstractmethod
    def update_raceline_planning_problem(
            self,
            raceline_problem
    ) -> None:
        """
        Updates the raceline planning problem
        :param raceline_problem: cr raceline planning problem
        """
        pass

    @abstractmethod
    def update_config(
            self,
            config: Any
    ) -> None:
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







