#!/bin/bash
update_type="update_type:"
knowledge="knowledge:"

# ADD WAYPOINT INSTANCES
for i in $(seq 0 9); do
update_type="$update_type
- 0";
knowledge="$knowledge
- knowledge_type: 0
  instance_type: 'waypoint'
  instance_name: 'wp$i'";
done

# ADD DOCK_AT
update_type="$update_type
- 0";
knowledge="$knowledge
- knowledge_type: 1
  attribute_name: 'dock_at'
  values:
  - {key: 'wp', value: 'wp0'}"

# ADD ROBOT_AT
update_type="$update_type
- 0";
knowledge="$knowledge
- knowledge_type: 1
  attribute_name: 'robot_at'
  values:
  - {key: 'wp', value: 'wp0'}"

rosservice call /rosplan_knowledge_base/update_array "
$update_type
$knowledge";
