# Launch URDF in Gazebo Fortress

## Start empty world

`ign gazebo empty.sdf`

### Check for service

* /world/empty/create

`ign service -l`

### Check for the request and response of the service

`ign service -is /world/empty/create`

## Spawm URDF model

`ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'`

* OBS: Pode-se usar XACRO para converte XACRO -> URDF e então carregar no gazebo. Fazer isso com um launcher.

## Create a bridge between topics

`ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@IGN_MSG`

* Mais informações: https://gazebosim.org/docs/fortress/ros2_integration#bidirectional-communication