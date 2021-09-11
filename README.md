
Question 2 - iiwa14Kine.py

I designed the code for all questions, which include forward kinematics, forward kinematics centre of mass, Jcobian, Jacobian_com, iterative inverse kinematics, Inverse kinematics, B(q), C(q, q_dot) and G(q). Furthermore, I made launch and pubilisher node files to check my results.

Question 5-(a)
I edited the launch file and made the publisher node to move the iiwa manipulator. The code run approximately a minute. My plot showed the acceleration of each joint when you edit some code. 
1st joint -> change integer to 0
2nd joint -> change integer to 1
.....
7th joint -> change integer to 6
You can see the acceleration of each joint seperately.


Question 5-(c)
I edited the launch file and made the publisher node as well. The code run approximately 70 seconds to finish all calculations, which include each mass at each configuration, average mass at those configurations, each translation x, y and z-axis distance between end-effector and centre of the mass, average distance between end-effector and centre of the mass, and finally the external torque. 
All results are reasonable when I compared with the theoritical values. Furthermore, all results are contained into the ROS Info file, which can prove this comment out in the terminal (command line).


It is important to notice that I changed the name of KDL file (kdl_kine_solver -> kdl_kine_solver_iiwa14)
I left the kdl_kine_solver for the youbot manipulator and I made a new file for the iiwa14 manipulator because I changed a little code to change the link name of iiwa manipulator. Therefore, If you use my launch file, the code will successfully run.

Thank you.

Kind regards
Hyungjoo Kim
