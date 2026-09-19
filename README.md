# Trying to get a robot arm to juggle
Viam's Fine Motor Skills Hackathon - September 2026! Mandy Meindersma, Jie Xuan Tang and Aika K grouped together to try to make a robot juggle. Much harder than we thought :P  

# Initial plan:
- One arm juggling
- 2 balls
- Throw one of in the air high enough and fast enough and then secretly load another ball into the arm for it to throw up. Then it can oscillate between throwing the balls
- Don't use any camera or vision what so ever, just try to throw and catch the ball in the same place every time
- Attach a bowl/funnel to the gripper of the arm to help with launching and catching


# Problem #0
The joint closest to the gripper (the "wrist" joint) moves so so so so slow. It can not launch a ball at all. 
</br>
**Solution:** Extend the arm and use the joint closest to the base of the arm that way only a small rotation is needed to make the gripper fly up. 

# Problem #1
Okay so we have the arm moving from position to position but this is too slow. It uses motion planning to compute the best path and avoids obstacles which means we can not launch the ball high enough.
</br>
**Solution:** Use joint movements instead

# Problem #2
Now that we have the joints moving it is going faster but no where near as fast as we want it. We have the arm just moving only the "shoulder" joint to throw the arm up
</br>
**Solution:** We can add a "flick" in the "elbow" joint. Means instead of going from arm position A to arm position B we are going to do:  A -> B -> C. This means using the method `move_through_joint_positions` rather than `move_to_joint_positions`

# Problem #3
The function `move_through_joint_positions` was not going fast enough. We were trying to figure out a way to be able to pass in the speed through an option but when talking to Viam they knew it was supported with go but they weren't sure if python supported it. It turns out they do support it, just on an unreleased version of the viam sdk. 
</br>
**Solution:** Build the source code of the viam sdk and then use that version to run the move through joint position. 

# Problem #4
The function `move_through_joint_positions` was still not going fast enough. That function does interpolation calculations between A -> B -> C so it is more like A -> a1 -> a2 -> a3 -> B -> b1 -> b2 -> b3 -> C. Stopping in that many places means that arms velocity or acceleration are not maxing out.
</br>
**Solution:** Clone the [Viam UFactory xArm Module](https://github.com/viam-modules/viam-ufactory-xarm) and delete the line of code that does the interpolation. `interpolate:  true,` -> `interpolate:  false,` in `xarm.go`. Then build your own viam module with these steps:
``` bash
brew install viam
viam login
viam module reload --part-id part_id_from_app.viam.com
```

# Problem #5
Now the arm is FINALLY going fast enough. Not fast enough to throw the ball, but fast enough to max out the speed of the arm to throw an error "Linear speed exceeds limit in ServoJ mode"
</br>
**Solution:** Hack into the IP of the arm to get to its admin panel (Viam provides the IP forwarding!) where we can mess with the settings and the max of the arm

# Problem #6
The admin panel doesn't let you change the max speed of the arm... It is a greyed out field that you can not touch hahaha
</br>
**Solution:** Write python code in the admin panel to hack the speed limit to be higher hahaha `arm.set_linear_spd_limit_factor(5)` which sets a max linear speed of 5m/s (1m/s * 5)

Now we finally have it launching it!!!!! It is hitting the ceiling!


# Final additions
At this point we had 3 hours left in the hackathon and we were going to try to see if we could get the arm to catch the ball but we realized we probably didn't have enough time. So we tried to think of other fun things to add. We decided to train an ML model to watch the funnel and if there was a ball placed, throw it. Do not throw if there is a hand in the way hahah. It worked so well! For the demo we were even able to throw the ball to Grant and he would catch it while juggling. So fun!