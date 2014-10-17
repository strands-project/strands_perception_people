## Logging package
This packge contains a logging node to save the detections to the message_store.

All the information given on how to run the nodes should only be used if you need to run them seperately. In normal cases please refer to the `perception_people_launch` package to start the whole perception pipeline.

### Logging
This node uses the `Logging.msg` to save the detected people together with their realworld position, the robots pose, the upper body detector and people tracker results, and the tf transform used to create the real world coordinates in the message store.

Run with:

`roslaunch bayes_people_tracker_logging logging.launch`

Parameters:
* `log`: _Default: true_ This convenience parameter allows to start the whole system without logging the data

