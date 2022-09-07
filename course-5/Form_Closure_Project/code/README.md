I use test cass similar to Example 12.7 and Figure 12.15

**Case 1:** Two fingers each touch two different edges of the hole, creating four contact normals.

$$F = [[0,0,-1,2]; [-1,0,1,0]; [0,-1,0,1]]$$

This grasp does yield form closure.

**Case 2:** If the right-hand finger were moved to the bottom right corner of the hole, the new F matrix

$$F = [[0,0,0,2]; [-1,0,1,0]; [0,-1,0,-1]]$$

This grasp does not yield form closure: the body can slide downward on the page.