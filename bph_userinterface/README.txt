1. Start rosbridge (if not already running):              
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9405
  (need custom port because the turtlebot is already using the default)
                                                                                
  2. Start the UI server (from the bph_userinterface/ directory):               
  python3 bph_ui_server.py                                                      
                                                                                
  Then open http://localhost:8080 in a browser. The human request display is at 
  http://localhost:8080/humanrequest.                                           
   
  Optional: pass a custom port as the first argument:                           
  python3 bph_ui_server.py 9000                             
                                                                                
  If you're accessing it from another machine on the network, use the robot's IP
   instead of localhost, and update the rosbridge URL in the connection bar to  
  match (e.g. ws://192.168.1.42:9090). The /humanrequest page will auto-detect  
  the hostname from the URL it was opened with, so it should just work if you   
  open http://robot-ip:8080/humanrequest directly.   

