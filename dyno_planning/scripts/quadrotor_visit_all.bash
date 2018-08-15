#!/bin/bash

echo "Adding problem from world state.";
rosservice call /dyno_plan/add_problem "{}"

echo "Calling problem generator.";
rosservice call /rosplan_problem_interface/problem_generation_server;

echo "Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;

echo "Calling plan parser.";
rosservice call /rosplan_parsing_interface/parse_plan;

echo "Calling plan dispatcher.";
rosservice call /rosplan_plan_dispatcher/dispatch_plan;

echo "Finished!";
