Origincar
Team: Once it's better, let's go party Team<br>
School: Harbin Institute of Technology (Weihai)<br>
Team Leader: Li Chengmou<br>
Model Used: RDK S100<br>
Competition Mode: Fully Automatic Mode<br>
Overview of Technical Approach and System Composition:<br>
1. Use nav2 for full-map navigation and obstacle avoidance, hand-drawn map<br>
2. Use WeChat OpenCV library for QR code scanning<br>
3. Large model deployed both on cloud and locally<br>

Start the camera driver, QR code decoder, navigation, chassis driver, map relocalization, and other nodes -> send the vehicle -> navigate to the task release point -> scan the QR code -> based on the QR code information, follow two different routes for the second phase of navigation -> return to point P.<br>

Regularly replan the route at a certain frequency, and when encountering obstacles, coordinate obstacle avoidance between the navigation controller's avoidance module and the replanned path.
