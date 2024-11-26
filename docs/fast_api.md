## Raceline Fast Api
The Raceline Fast API allows users with only a few lines of code to generate the CommonRoad Raceline objects from a 
CommonRoad scenario and a CommonRoad planning problem using the different planners.

## Resources 
You can find the .ini and the ggv file [here](https://github.com/TUMFTM/global_racetrajectory_optimization/tree/master/inputs).



### Example 1: Shortest Path Planner


```Python
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_raceline_planner.raceline import RaceLine
from commonroad_raceline_planner.raceline_fast_api import generate_ftm_shortest_path_raceline_from_cr

scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/YOUR/SCENARIO").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

raceline: RaceLine = generate_ftm_shortest_path_raceline_from_cr(
                    cr_scenario=scenario,
                    planning_problem=planning_problem,
                    ini_path="PATH/TO/.ini",
                    ggv_file="PATH/TO/ggv",
                    ax_max_machines_file="PATH/TO/ENGINECONSTRAINTS",
                    show_plot=True
                )
```


### Example 2: Minimum Curvature Planner

```Python
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_raceline_planner.raceline import RaceLine
from commonroad_raceline_planner.raceline_fast_api import generate_ftm_minimum_curvature_raceline_from_cr

scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/YOUR/SCENARIO").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

raceline: RaceLine = generate_ftm_minimum_curvature_raceline_from_cr(
                    cr_scenario=scenario,
                    planning_problem=planning_problem,
                    ini_path="PATH/TO/.ini",
                    ggv_file="PATH/TO/ggv",
                    ax_max_machines_file="PATH/TO/ENGINECONSTRAINTS",
                    show_plot=True
                )
```


### Example 3: Saving the output

```Python
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_raceline_planner.raceline import RaceLine
from commonroad_raceline_planner.raceline_fast_api import generate_ftm_shortest_path_raceline_from_cr

scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/YOUR/SCENARIO").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

raceline: RaceLine = generate_ftm_shortest_path_raceline_from_cr(
                    cr_scenario=scenario,
                    planning_problem=planning_problem,
                    ini_path="PATH/TO/.ini",
                    ggv_file="PATH/TO/ggv",
                    ax_max_machines_file="PATH/TO/ENGINECONSTRAINTS",
                    traj_race_export="PATH/TO/SAVE/TRAJECTORY",
                    velocity_profile_export="PATH/TO/SAVE/VELOCITY_PROFILE",
                    show_plot=True
                )
```