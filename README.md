Configs can be changed through modifing the `src/user_config/user_config.yaml`. When run `main.sh`, the python script will re-generated `*.launch`, `*.world` and so on, according to configs in `user_config.yaml`.

Plugins: `src/plugins`
Config: `src/user_config`

Pathfinding Algorithm:
`src/core/path_planner/path_planner/include`
`src/core/path_planner/path_planner/src`


Bellman_ford:
`src/core/path_planner/path_planner/include/path_planner/graph_planner/bellman_ford.h`
`src/core/path_planner/path_planner/src/graph_planner/bellman_ford.cpp`