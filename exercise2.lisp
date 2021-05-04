(defun move-bottle2 (bottle-spawn-pose)
  (spawn-object bottle-spawn-pose)
  (with-simulated-robot
      (cpl:par
        ;; Moving the robot near the table.
        
        (perform (a motion
                    (type moving-torso) 
                    (joint-angle 0.3)))
        (park-arms))
 
    (let ((?perceived-bottle (perceive-bottle))
 
          (?grasping-arm :right))
      ;; We update the value of ?grasping-arm according to what the method used
      (setf ?grasping-arm (pick-up-object ?perceived-bottle ?grasping-arm))
      (park-arm ?grasping-arm)
      ;; Moving the robot near the counter.
      (let ((?nav-goal *base-pose-near-counter*))
        (perform (an action
                     (type going)
                     (target (a location 
                                (pose ?nav-goal))))))
       ;; Setting the object down on the counter
      (let ((?drop-pose *final-object-destination*))
        (perform (an action
                     (type placing)
                     (arm ?grasping-arm)
                     (object ?perceived-bottle)
                     (target (a location 
                                (pose ?drop-pose))))))
      (park-arm ?grasping-arm))))