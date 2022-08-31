this package uses [catkin_simple](https://github.com/catkin/catkin_simple), so be sure to clone it in your workspace before attempting to compile this package.

**NOTE:**: simple translation of the message is not possible when using PX4 internal controller, since the waypoints need to be sent at a slower frquency for the controller to execute them. 
This package does not provide a queueing mechanism, therefore it is not suitable for controlled 
