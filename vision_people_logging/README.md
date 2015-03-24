# Vision/People Logging package

This packge contains a logging node to save the detections to the message_store.

All the information given on how to run the nodes should only be used if you need to run them seperately. In normal cases please refer to the `perception_people_launch` package to start the whole perception pipeline.

## Logging

This node uses the `LoggingUBD.msg` to save the detected people together with the robots pose and the tf transform which can be used to create the real world coordinates in the message store.

Run with:

`roslaunch vision_people_logging logging_ubd.launch`

Parameters:
* `log`: _Default: true_ This convenience parameter allows to start the whole system without logging the data

