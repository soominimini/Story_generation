# Guidline for use of the robot
First, start the riva server to recognize the speech. 
``` 
cd ~/robot/riva_quickstart_arm64_v2.14.0
bash ./riva_start.sh ./config.sh -s
roslaunch qt_riva_asr_app qt_riva_asr_app.launch 
```
Second, open pycharm IDE go to directory of /home/qtrobot/tutorials-master/demos/version_1_llm
run web_user_server.py under src folder
